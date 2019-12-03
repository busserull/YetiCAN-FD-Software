defmodule Yeti.Device.Config do
  import Ecto.Changeset

  defp config_format do
    %{
      nominal_1: :integer,
      nominal_2: :integer,
      data_1: :integer,
      data_2: :integer,

      tef_depth: :integer,
      tef_timestamp: :boolean,

      txq_depth: :integer,
      txq_payload: :integer
    }
  end

  defp user_params do
    [
      :nominal_1, :nominal_2, :data_1, :data_2,
      :tef_depth, :tef_timestamp,
      :txq_depth, :txq_payload
    ]
  end



  def changeset(config, params) do
    {config, config_format()}
    |> cast(params, user_params())
    |> validate_required(user_params())
    |> validate_inclusive_range(:nominal_1, 0, 255)
    |> validate_inclusive_range(:nominal_2, 0, 127)
    |> validate_inclusive_range(:data_1, 0, 31)
    |> validate_inclusive_range(:data_2, 0, 15)
    |> validate_inclusive_range(:tef_depth, 1, 32)
    |> validate_inclusive_range(:txq_depth, 1, 32)
    |> validate_payload_size(:txq_payload)
  end


  defp validate_inclusive_range(config, field, lower, upper) do
    validate_change config, field, fn field, value ->
      if value < lower || value > upper do
        [{field, "must be in range [#{lower}, #{upper}]"}]
      else
        []
      end
    end
  end

  defp validate_payload_size(config, field) do
    valid_payload_sizes = [8, 12, 16, 20, 24, 32, 48, 64]

    validate_change config, field, fn field, size ->
      if size in valid_payload_sizes do
        []
      else
        [{field, "must be in {#{Enum.join valid_payload_sizes, ", "}}"}]
      end
    end
  end

end
