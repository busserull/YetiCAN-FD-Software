import {Socket} from "phoenix"

let socket = new Socket("/socket", {params: {}})

socket.connect()

let channel = socket.channel("device", {})
let sendButton = document.querySelector("#can-send")
let messagesContainer = document.querySelector("#messages")

sendButton.addEventListener("click", () => {
    channel.push("send", {message: "Yeti"})
})

channel.on("message", payload => {
    let messageItem = document.createElement("li")
    messageItem.innerText = `${payload.message}`
    messagesContainer.appendChild(messageItem)
})

channel.join()
    .receive("ok", resp => { console.log("Joined successfully", resp) })
    .receive("error", resp => { console.log("Unable to join", resp) })

export default socket
