defmodule YetiWeb.ConfigController do
  use YetiWeb, :controller

  def new(conn, _params) do
    changeset = Yeti.Device.get_config()
    render(conn, "new.html", changeset: changeset)
  end

  def create(conn, %{"config" => config}) do
    case Yeti.Device.update_config(config) do
      {:ok, _changeset} ->
        conn
        |> put_flash(:info, "Config Commited")
        |> redirect(to: Routes.page_path(conn, :index))

      {:error, changeset} ->
        render(conn, "new.html", changeset: changeset)
    end
  end

  defp get_params(map, params) do
    Enum.reduce params, %{}, fn key, acc ->
      string_key = Atom.to_string(key)
      value = Map.get(map, string_key)

      Map.put(acc, key, value)
    end
  end

end
