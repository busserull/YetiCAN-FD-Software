import {Socket} from "phoenix"

let socket = new Socket("/socket", {params: {}})

socket.connect()

let channel = socket.channel("device", {})
let sendInput = document.querySelector("#can-input")
let sendButton = document.querySelector("#can-send")
let messagesContainer = document.querySelector("#messages")

function onlyValidBytes(string){
    let bytes = string.trim().split(" ");

    let isHex = string => {
        return RegExp('^[0-9a-fA-F]{2}$').test(string);
    }

    return bytes.every(isHex)
}

sendInput.addEventListener("input", () => {
    if(onlyValidBytes(sendInput.value.trim())){
        sendInput.style.color = "#393e46";
    }
    else{
        sendInput.style.color = "#fa4252";
    }
})

sendButton.addEventListener("click", () => {
    let message = sendInput.value.trim();

    if(onlyValidBytes(message)){
        channel.push("send", {message: message})
    }
    else{
        console.log("Animation start");
        sendButton.classList.add("apply-shake");
    }
})

sendButton.addEventListener("animationend", () => {
    sendButton.classList.remove("apply-shake");
    console.log("Animation end");
})

channel.on("message", payload => {
    console.log(`Raw payload: ${payload}`)

    let messageItem = document.createElement("table");
    messageItem.style.borderCollapse = "collapse";
    messageItem.style.borderSpacing = "0px";
    messageItem.style.borderStyle = "solid";
    messageItem.style.padding = "10px 5px";
    messageItem.style.borderWidth = "1px";
    messageItem.overflow = "hidden";
    messageItem.wordBreak = "normal";
    messageItem.borderColor = "black";

    let headerRow = document.createElement("tr");
    let headerElements = [];
    for(let i = 0; i < 7; i++){
        let element = document.createElement("td");
        element.style.borderStyle = "solid";
        element.style.borderWidth = "1px";
        element.style.borderBottom = "1px";
        element.style.paddingLeft = "10px";
        element.style.paddingRight = "10px";
        headerElements.push(element);
    }

    headerElements[0].innerHTML = `ID ${payload.message.frame_id}`;
    headerElements[1].innerHTML = `Filter ${payload.message.filter_hit}`;
    if(payload.message.use_fd_format){
        headerElements[2].innerHTML = "CAN FD Format";
    }
    else{
        headerElements[2].innerHTML = "CAN 2.0 Format";
    }
    if(payload.message.use_bit_rate_switch){
        headerElements[3].innerHTML = "Bit Rate Switch";
    }
    else{
        headerElements[3].innerHTML = "Nominal Rate";
    }
    if(payload.message.use_extended_id){
        headerElements[4].innerHTML = "Extended ID";
    }
    else{
        headerElements[4].innerHTML = "Normal ID";
    }
    if(payload.message.error_active){
        headerElements[5].innerHTML = "Error Active";
    }
    else{
        headerElements[5].innerHTML = "No Error";
    }
    if(payload.message.timestamp_valid){
        headerElements[6].innerHTML = `Timestamp ${payload.message.timestamp}`;
    }
    else{
        headerElements[6].innerHTML = "No Timestamp";
    }
    headerElements.forEach( el => {headerRow.appendChild(el)} );


    let dataRow = document.createElement("tr");
    let dataElements = [];
    for(let i = 0; i < 2; i++){
        let element = document.createElement("td");
        element.style.borderStyle = "solid";
        element.style.borderWidth = "1px";
        element.style.borderBottom = "1px";
        element.style.paddingLeft = "10px";
        element.style.paddingRight = "10px";
        dataElements.push(element);
    }
    dataElements[1].setAttribute("colspan", "6");

    dataElements[0].innerHTML = `${payload.message.data_length} bytes`;
    dataElements[1].innerHTML = `${payload.message.data}`

    dataElements.forEach( el => {dataRow.appendChild(el)} );


    messageItem.appendChild(headerRow);
    messageItem.appendChild(dataRow);
    messagesContainer.appendChild(messageItem);

})

channel.join()
    .receive("ok", resp => { console.log("Joined successfully", resp) })
    .receive("error", resp => { console.log("Unable to join", resp) })

export default socket
