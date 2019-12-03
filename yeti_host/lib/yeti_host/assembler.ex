defmodule Yeti.Assembler do
  use GenServer

  @sentinel_stx                       0x02
  @sentinel_etx                       0x03

  @type_config_commit                 0x00
  @type_config_bit_rate               0x01
  @type_config_transmit_event_fifo    0x02
  @type_config_transmit_queue         0x03
  @type_config_fifo                   0x04
  @type_config_filter                 0x05

  @type_send_from_host                0x20
  @type_send_to_host                  0x30

  @type_response                      0x80

  @response_ok                        0x00

  def start_link(opts \\ [name: __MODULE__]) do
    GenServer.start_link __MODULE__, "ttyACM0", opts
  end

  def send(message) do
    GenServer.call __MODULE__, {:send, message}
  end

  ### Callbacks ###

  def init(port) do
    {:ok, uart_pid} = Circuits.UART.start_link()

    Circuits.UART.open(uart_pid, port, active: true)

    {:ok, uart_pid}
  end

  def handle_call({:send, message}, _from, uart_pid) do
    Circuits.UART.write(uart_pid, message)
    {:reply, :ok, uart_pid}
  end

  def handle_info({:circuits_uart, _port, bytes}, uart_pid) do
    IO.puts "Bytes: #{inspect bytes}"
    # YetiWeb.Endpoint.broadcast("device", "message", %{message: "#{inspect bytes}"})
    case parse(bytes) do
      # {:response, :ok} ->
      #   display("ACK")

      {:send_to_host, response} ->
        YetiWeb.Endpoint.broadcast("device", "message", %{message: response})
        # display("Payload: #{inspect response.data}")

      # {:error, type} ->
      #   display("Error: #{Atom.to_string(type)}")

      _ ->
        :ok
    end

    {:noreply, uart_pid}
  end

  ### Helpers ###

  defp parse(message) do
    {head, payload, crc, tail} = split_message(message)

    case {head, tail} do
      {<< @sentinel_stx >>, << @sentinel_etx >>} ->
        cond do
          :erlang.crc32(payload) == :binary.decode_unsigned(crc, :big) ->
            parse_type(payload)

          true ->
            {:error, :crc}
        end

      _ ->
        {:error, :stx_etx}
    end
  end

  defp parse_type(<< type :: binary-size(1), payload :: binary >>) do
    case :binary.first(type) do
      @type_response ->
        parse_response(payload)

      @type_send_to_host ->
        parse_send_to_host(payload)

      _ ->
        {:error, :type}
    end
  end
  defp parse_type(_) do
    {:error, :type}
  end

  defp parse_response(<< @response_ok >>) do
    {:response, :ok}
  end
  defp parse_response(_) do
    {:response, :fault}
  end

  def parse_send_to_host(payload) do
    <<
      raw_use_fd_format :: binary-size(1),
      raw_use_bit_rate_switch :: binary-size(1),
      raw_use_extended_id :: binary-size(1),
      raw_error_active :: binary-size(1),
      raw_frame_id :: binary-size(4),
      raw_filter_hit :: binary-size(1),
      raw_timestamp_valid :: binary-size(1),
      raw_timestamp :: binary-size(4),
      raw_data_length :: binary-size(1),
      raw_data :: binary
    >> = payload

    use_fd_format = raw_use_fd_format == << 1 >>
    use_bit_rate_switch = raw_use_bit_rate_switch == << 1 >>
    use_extended_id = raw_use_extended_id == << 1 >>
    error_active = raw_error_active == << 1 >>

    frame_id =
      raw_frame_id
      |> :binary.decode_unsigned(:big)
      |> Integer.to_string(16)
      |> String.pad_leading(8, "0")

    filter_hit = :binary.decode_unsigned(raw_filter_hit, :big)
    timestamp_valid = raw_timestamp_valid == << 1 >>
    timestamp = :binary.decode_unsigned(raw_timestamp, :big)
    data_length = :binary.decode_unsigned(raw_data_length, :big)

    data =
      raw_data
      |> :binary.bin_to_list()
      |> Enum.map(fn byte -> Integer.to_string(byte, 16) end)
      |> Enum.map(fn hex -> String.pad_leading(hex, 2, "0") end)

    response = %{
      use_fd_format: use_fd_format,
      use_bit_rate_switch: use_bit_rate_switch,
      use_extended_id: use_extended_id,
      error_active: error_active,
      frame_id: frame_id,
      filter_hit: filter_hit,
      timestamp_valid: timestamp_valid,
      timestamp: timestamp,
      data_length: data_length,
      data: data
    }

    {:send_to_host, response}
  end



  defp split_message(message) do
    payload_length = byte_size(message) - 6

    if payload_length < 0 do
      {nil, nil, nil, nil}
    else
      <<
        head :: binary-size(1),
        payload :: binary-size(payload_length),
        crc :: binary-size(4),
        tail :: binary
      >> = message

      {head, payload, crc, tail}
    end
  end


  defp display(message) do
    YetiWeb.Endpoint.broadcast("device", "message", %{message: message})
  end

end
