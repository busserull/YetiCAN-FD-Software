defmodule Yeti.Device.ConfigTracker do
  use Agent

  def start_link(initial_config) do
    Agent.start_link fn -> initial_config end, name: __MODULE__
  end

  def get do
    Agent.get __MODULE__, fn config -> config end
  end

  def update(new_config) do
    Agent.update __MODULE__, fn _config -> new_config end
  end

end
