#!/usr/bin/env python3

from ras_transport.interfaces.TransportWrapper import TransportFileServer
import rclpy
def main(args=None):
    rclpy.init(args=args)
    ftp_server = TransportFileServer()
    
    try:
        ftp_server.file_server.connect()
        ftp_server.file_server.serve()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect
        ftp_server.file_server.safe_kill()
        rclpy.shutdown()

if __name__ == '__main__':
    main()