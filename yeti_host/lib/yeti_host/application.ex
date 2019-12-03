defmodule Yeti.Application do
  # See https://hexdocs.pm/elixir/Application.html
  # for more information on OTP Applications
  @moduledoc false

  use Application

  def start(_type, _args) do
    initial_device_config = Application.get_env(
      :yeti_host, :initial_device_config
    )
    # List all child processes to be supervised
    children = [
      # Start the endpoint when the application starts
      YetiWeb.Endpoint,
      # Starts a worker by calling: Yeti.Worker.start_link(arg)
      # {Yeti.Worker, arg},
      {Yeti.Device.ConfigTracker, initial_device_config},
      {Yeti.Assembler, [name: Yeti.Assembler]}
    ]

    # See https://hexdocs.pm/elixir/Supervisor.html
    # for other strategies and supported options
    opts = [strategy: :one_for_one, name: Yeti.Supervisor]
    Supervisor.start_link(children, opts)
  end

  # Tell Phoenix to update the endpoint configuration
  # whenever the application is updated.
  def config_change(changed, _new, removed) do
    YetiWeb.Endpoint.config_change(changed, removed)
    :ok
  end
end
