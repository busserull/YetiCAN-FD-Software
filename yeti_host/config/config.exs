# This file is responsible for configuring your application
# and its dependencies with the aid of the Mix.Config module.
#
# This configuration file is loaded before any dependency and
# is restricted to this project.

# General application configuration
use Mix.Config

config :yeti_host, initial_device_config: %{
    nominal_1: 31,
    nominal_2: 9,
    data_1: 31,
    data_2: 9,
    tef_depth: 2,
    tef_timestamp: true,
    txq_depth: 1,
    txq_payload: 64
  }

config :yeti_host,
  namespace: Yeti

# Configures the endpoint
config :yeti_host, YetiWeb.Endpoint,
  url: [host: "localhost"],
  secret_key_base: "VNyg79b24q0ywISM+E2Opgtvbmh7V8eVBAohqSj7l8hcDg4tiIij0Hte15QDXk1/",
  render_errors: [view: YetiWeb.ErrorView, accepts: ~w(html json)],
  pubsub: [name: Yeti.PubSub, adapter: Phoenix.PubSub.PG2]

# Configures Elixir's Logger
config :logger, :console,
  format: "$time $metadata[$level] $message\n",
  metadata: [:request_id]

# Use Jason for JSON parsing in Phoenix
config :phoenix, :json_library, Jason

# Import environment specific config. This must remain at the bottom
# of this file so it overrides the configuration defined above.
import_config "#{Mix.env()}.exs"
