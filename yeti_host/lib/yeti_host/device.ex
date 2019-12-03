defmodule Yeti.Device do
  alias Yeti.Device.Config
  alias Yeti.Device.ConfigTracker

  def get_config do
    config = ConfigTracker.get()
    Config.changeset(config, %{})
  end

  def update_config(config) do
    case Config.changeset(%{}, config) do
      %{valid?: false} = changeset ->
        {:error, Map.put(changeset, :action, :update)}

      changeset ->
        ConfigTracker.update(changeset.changes)
        {:ok, changeset}
    end
  end

  def send_message(payload) do
    raw_payload =
      payload
      |> String.split()
      |> Enum.map(&(String.to_charlist(&1)))
      |> Enum.map(&(:erlang.list_to_integer(&1, 16)))
      |> :binary.list_to_bin()

    data_length = << byte_size(raw_payload) >>

    message_body = <<
      0x20,
      0x01, 0x00, 0x01, 0x00,
      0x00, 0x00, 0xc0, 0xde,
      0x00, 0x00, 0x00, 0x01
    >>
    <> data_length
    <> raw_payload

    crc =
      message_body
      |> :erlang.crc32()
      |> :binary.encode_unsigned(:big)

    message = <<0x02>> <> message_body <> crc <> <<0x03>>

    Yeti.Assembler.send(message)
  end


end
