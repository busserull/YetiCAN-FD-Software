defmodule YetiWeb.DeviceChannel do
  use Phoenix.Channel

  def join("device", _message, socket) do
    {:ok, socket}
  end

  def handle_in("send", %{"message" => payload}, socket) do
    Yeti.Device.send_message(payload)
    {:noreply, socket}
  end
end
