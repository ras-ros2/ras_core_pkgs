from ..TrasnportInterfaces import PublisherInterface,SubscriberInterface,FileClientInterface,FileServerInterface
from pathlib import Path
from typing import Callable
import paho
import paho.mqtt.client
import paho.mqtt.publish
from paho.mqtt.enums import CallbackAPIVersion

from pyftpdlib.authorizers import DummyAuthorizer
from pyftpdlib.handlers import FTPHandler
from pyftpdlib.servers import FTPServer
from ftplib import FTP

import os

class MqttPublisher(PublisherInterface):
    def __init__(self, topic: str, ip: str, port: int) -> None:
        self.topic = topic
        self.ip = ip
        self.port = port
        self.client = paho.mqtt.client.Client(
            CallbackAPIVersion.VERSION2,
            client_id="",
            userdata={},
            protocol=paho.mqtt.client.MQTTv311,
            transport="tcp",
            reconnect_on_failure=True,
        )
        self.client.enable_logger()
        self.client.on_publish = paho.mqtt.publish._on_publish
        self.client.on_connect = paho.mqtt.publish._on_connect

    def connect(self) -> None:
        if self.is_connected():
            return
        self.client.connect(self.ip, self.port, keepalive=60)

    def publish(self, msg: bytes) -> None:
        if not self.is_connected():
            self.connect()
        self.client.publish(self.topic, msg, qos=1)

    def disconnect(self) -> None:
        if not self.is_connected():
            return
        self.client.disconnect()
    
    def is_connected(self) -> bool:
        return self.client.is_connected()
    
class MqttSubscriber(SubscriberInterface):
    def __init__(self, topic: str, ip: str, port: int, callback: Callable[[bytes], None]) -> None:
        self.topic = topic
        self.ip = ip
        self.port = port
        self.callback = callback
        self.client = paho.mqtt.client.Client(
            CallbackAPIVersion.VERSION2,
            client_id="",
            userdata={},
            protocol=paho.mqtt.client.MQTTv311,
            transport="tcp",
            reconnect_on_failure=True,
        )
        self.client.enable_logger()
        self.client.on_message = self.on_message
        # self.client.on_connect = self.on_connect

    def connect(self) -> None:
        if self.is_connected():
            return
        self.client.connect(self.ip, self.port, keepalive=60)
        self.client.subscribe(self.topic)

    def on_message(self,mosq, obj, msg):
        mosq.publish('pong', 'ack', 0)
        self.callback(msg.payload)

    # def on_connect(self, client, userdata, flags, rc):
    #     print("Connected with result code "+str(rc))

    def callback(self, msg):
        pass

    def disconnect(self) -> None:
        if not self.is_connected():
            return
        self.client.disconnect()
    
    def is_connected(self) -> bool:
        return self.client.is_connected()
    
class FtpServer(FileServerInterface):
    def __init__(self, path: Path, ip: str, port: int) -> None:
        self.path = Path(path)
        assert self.path.exists()
        self.ip = ip
        self.port = port
        self.server = None

    def connect(self) -> None:
        authorizer = DummyAuthorizer()
        authorizer.add_user("user", "password", str(self.path), perm="elradfmw")
        authorizer.add_anonymous(str(self.path))

        handler = FTPHandler
        handler.authorizer = authorizer

        self.server = FTPServer((self.ip, self.port), handler)

    def serve(self) -> None:
        if not self.is_connected():
            self.connect()
        else:
            self.server.serve_forever()
    
    def disconnect(self) -> None:
        if not self.is_connected():
            return
        self.server.close_all()
        self.server = None
    
    def is_connected(self) -> bool:
        return self.server is not None

    def safe_kill(self) -> None:
        self.disconnect()

class FtpClient(FileClientInterface):
    def __init__(self, ip: str, port: int) -> None:
        self.ip = ip
        self.port = port
        self.ftp = FTP()
    
    def connect(self) -> None:
        if self.is_connected():
            return
        self.ftp.connect(self.ip, self.port)
        self.ftp.login("user", "password")
        self.ftp.cwd("/")
    
    def disconnect(self) -> None:
        if not self.is_connected():
            return
        self.ftp.quit()
    
    def is_connected(self) -> bool:
        return self.ftp.sock is not None
    
    def download(self, remote_path: Path, local_path: Path) -> None:
        if not self.is_connected():
            self.connect()
        local_path = Path(local_path)
        with local_path.open("wb") as f:
            self.ftp.retrbinary("RETR " + str(remote_path), f.write)
        
    def upload(self, local_path: Path, remote_path: Path) -> None:
        if not self.is_connected():
            self.connect()
        local_path = Path(local_path)
        with local_path.open("rb") as f:
            self.ftp.storbinary("STOR " + str(remote_path), f)

    def safe_kill(self) -> None:
        self.disconnect()
    
from flask import Flask, request, send_from_directory, render_template_string, jsonify
     
class HttpServer(FileServerInterface):
    def __init__(self, path: Path, ip: str, port: int) -> None:
        self.path = Path(path)
        assert self.path.exists()
        self.ip = ip
        self.port = port
        self.server = None
        self.app = Flask(__name__)
        self.app.config['SERVE_FOLDER'] = str(self.path.resolve().absolute())

    def connect(self) -> None:
        def home():
            files = [ str(_path) for _path in self.path.glob("*") ]
            return render_template_string('''
                <h1>File Upload and Download</h1>
                <h2>Upload a File</h2>
                <form id="uploadForm" method="POST" action="/upload" enctype="multipart/form-data">
                    <input type="file" name="file">
                    <button type="submit">Upload</button>
                </form>
                <div id="message"></div>
                <h2>Download Files</h2>
                <ul id="fileList">
                    {% for file in files %}
                        <li><a href="/download/{{ file }}">{{ file }}</a></li>
                    {% endfor %}
                </ul>
                <script>
                    const form = document.getElementById('uploadForm');
                    form.addEventListener('submit', async (e) => {
                        e.preventDefault();
                        const formData = new FormData(form);
                        const response = await fetch('/upload', {
                            method: 'POST',
                            body: formData
                        });
                        const result = await response.json();
                        document.getElementById('message').innerText = result.message;
                        if (result.success) {
                            const fileList = document.getElementById('fileList');
                            fileList.innerHTML = '';
                            result.files.forEach(file => {
                                const li = document.createElement('li');
                                const link = document.createElement('a');
                                link.href = `/download/${file}`;
                                link.innerText = file;
                                li.appendChild(link);
                                fileList.appendChild(li);
                            });
                        }
                    });
                </script>
            ''', files=files)

        @self.app.route('/upload', methods=['POST'])
        def upload_file():
            if 'file' not in request.files:
                return jsonify({"success": False, "message": "No file part"}), 400
            file = request.files['file']
            if file.filename == '':
                return jsonify({"success": False, "message": "No selected file"}), 400
            file.save(os.path.join(self.app.config['SERVE_FOLDER'], file.filename))
            files = [ str(_path) for _path in self.path.glob("*") ]
            return jsonify({"success": True, "message": "File uploaded successfully", "files": files})

        @self.app.route('/download/<filename>')
        def download_file(filename):
            return send_from_directory(self.app.config['SERVE_FOLDER'], filename)


    def serve(self) -> None:
        if not self.is_connected():
            self.connect()
        else:
            self.app.run(port=self.port)
    
    def disconnect(self) -> None:
        if not self.is_connected():
            return
        print("Shutting down server...")
        func = request.environ.get('werkzeug.server.shutdown')
        if func is None:
            raise RuntimeError('Not running with the Werkzeug Server')
        func()
        # if self.server_thread:
        #     self.server_thread.join()
        self.server = None
    
    def is_connected(self) -> bool:
        return self.server is not None

    def safe_kill(self) -> None:
        self.disconnect()

import os
import requests

class HttpClient(FileClientInterface):
    def __init__(self, ip: str, port: int) -> None:
        self.ip = ip
        self.port = port
        self.server_url = f"http://{ip}:{port}"
        self.connected = False
    
    def connect(self) -> None:
        if self.is_connected():
            return
        try:
            requests.head(self.server_url)
        except requests.ConnectionError:
            raise RuntimeError(f"Can't connect to {self.server_url}")
        self.connected = True
    
    def disconnect(self) -> None:
        if not self.is_connected():
            return
        self.connected = False
    
    def is_connected(self) -> bool:
        return self.connected
    
    def download(self, remote_path: Path, local_path: Path) -> None:
        if not self.is_connected():
            self.connect()
        response = requests.get(f"{self.server_url}/download/{remote_path}", stream=True)
        if response.status_code == 200:
            with open(local_path, 'wb') as f:
                for chunk in response.iter_content(chunk_size=8192):
                    f.write(chunk)
            print(f"File {remote_path} downloaded successfully to {local_path}")
        else:
            print(f"Failed to download file {remote_path}:", response.status_code)
        
    def upload(self, local_path: Path, remote_path: Path) -> None:
        if not self.is_connected():
            self.connect()
        local_path = Path(local_path)
        if not local_path.exists():
            raise FileNotFoundError(f"File {local_path} does not exist.")
        with local_path.open('rb') as f:
            files = {'file': f}
            response = requests.post(f"{self.server_url}/upload", files=files)
            if response.status_code == 200:
                print("Upload successful:", response.json())
            else:
                print("Failed to upload file:", response.json())

    def safe_kill(self) -> None:
        self.disconnect()