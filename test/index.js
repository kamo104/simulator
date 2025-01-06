const ws = new WebSocket("ws://127.0.0.1:42003?clientId=tmpClient")

ws.addEventListener("message", (event) => {
  console.log("Message from server ", event.data);
});

ws.addEventListener("connect", (event) => {
  console.log("connected")
});
