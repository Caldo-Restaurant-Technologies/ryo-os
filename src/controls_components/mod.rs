pub mod motor;
pub mod units;
pub mod helper;
pub mod linear_actuator;
pub mod controller;

//pub type SharedSender = Arc<Mutex<mpsc::Sender<Message>>>;

// pub trait Write{
//     async fn write(&self, buffer: &[u8]) -> Result<Vec<u8>, Error>{
//         let (resp_tx, resp_rx) = oneshot::channel();
//         let msg = Message{
//             msg:buffer.to_vec(),
//             response: resp_tx
//         };
//         self.sender.lock().await.send(msg).await.expect("Channel was dropped");
//         let res = resp_rx.await.unwrap();
//         Ok(res)
//     }
// }