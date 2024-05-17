use tokio::io::{AsyncWriteExt, AsyncReadExt};
use tokio::net::{TcpStream, ToSocketAddrs};
use tokio::sync::{mpsc, oneshot};
use std::error::Error;
use log::{error, info};


pub struct Message {
    pub buffer: Vec<u8>,
    pub response: oneshot::Sender<Vec<u8>>,
}


pub async fn client<T: ToSocketAddrs>(addr: T,  mut msg: mpsc::Receiver<Message>) -> Result<(), Box<dyn Error + Send + Sync>> {
    let mut stream = TcpStream::connect(addr).await?;
    while let Some(message) = msg.recv().await {
        info!("Message Buffer: {:?}", &message.buffer);
        stream.write_all(&message.buffer).await?;
        stream.readable().await?;
        let mut buffer = [0; 100];
        match stream.read(&mut buffer).await {
            Ok(0) => {
                error!("Connection closed by server");
            }
            Ok(_) => {
                if message.response.send(buffer.to_vec()).is_err() {
                    error!("Failed to send via channel");
                }
            }
            Err(e) => {
                error!("Failed to read from stream: {}", e);
                break;
            }
        }
    }
    Ok(())
}

pub async fn echo_client(mut recv_channel: mpsc::Receiver<Message>) {
    while let Some(msg) = recv_channel.recv().await {
        //launch  cycle task
        if msg.response.send(msg.buffer).is_err() {
            eprintln!("Unable to send response");
            error!("Response send failed");
        }
    }
}

#[tokio::test]
async fn test_client() {
    let (tx, rx) = mpsc::channel(100);

    let client = tokio::spawn(
        client("192.168.1.11:8888", rx)
    );
    
    let controller = tokio::spawn(async move {
        let (resp_tx, resp_rx) = oneshot::channel();
        let command = vec![2, b'M',b'0',b'G',b'S',13];
        let cmd = Message {
            buffer:command,
            response:resp_tx
        };
        tx.send(cmd).await.expect("Shits broken");
        let response = resp_rx.await;
        match response {
            Ok(_) => {}
            Err(err) => {
                eprintln!("Error: {err}");
            }
        };
    });

    let _ = client.await;
    
    let _ = controller.await;
    
}


