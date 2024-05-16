use std::convert::Infallible;
use std::net::SocketAddr;
use tokio::sync::{mpsc};

use http_body_util::Full;
use hyper::body::Bytes;
use hyper::server::conn::http1;
use hyper::service::service_fn;
use hyper::{Request, Response};
use hyper_util::rt::TokioIo;
use tokio::net::{TcpListener};

pub mod controls_components;
pub mod tcp_client;


#[allow(dead_code)]
#[derive(Debug, Clone)]
enum HmiState {
    Start,
    Stop,
}

async fn hello(
    _: Request<hyper::body::Incoming>,
    tx: mpsc::Sender<HmiState>,
) -> Result<Response<Full<Bytes>>, Infallible> {
    tx.send(HmiState::Start).await.unwrap();
    Ok(Response::new(Full::new(Bytes::from("Hello, World!"))))
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let (tx, rx) = mpsc::channel(1);
    let server = tokio::spawn(ui_server(tx));
    let sys_controls = tokio::spawn(ui_request_handler(rx));
   
    sys_controls.await.unwrap();
    server.await.unwrap()
}

async fn ui_server(
    tx: mpsc::Sender<HmiState>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let addr = SocketAddr::from(([127, 0, 0, 1], 3000));
    // We create a TcpListener and bind it to 127.0.0.1:3000
    let listener = TcpListener::bind(addr).await?;
    loop {
        let (stream, _) = listener.accept().await?;
        let io = TokioIo::new(stream);
        // Spawn a tokio task to serve multiple connections concurrently
        let tx = tx.clone();
        tokio::task::spawn(async move {
            // Finally, we bind the incoming connection to our `hello` service
            if let Err(err) = http1::Builder::new()
                // `service_fn` converts our function in a `Service`
                .serve_connection(io, service_fn(|req| hello(req, tx.clone())))
                .await
            {
                eprintln!("Error serving connection: {:?}", err);
            }
        });
    }
}
async fn ui_request_handler(mut ui_state: mpsc::Receiver<HmiState>) {
    while let Some(state) = ui_state.recv().await {
        match state {
            HmiState::Start => {
                println!("Start Command Sent")
            }
            HmiState::Stop => {
                println!("Start Command Sent")
            }
        }
    }
}


