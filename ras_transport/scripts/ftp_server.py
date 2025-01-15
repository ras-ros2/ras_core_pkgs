#!/usr/bin/env python3

from ras_transport.interfaces.TransportWrapper import TransportFileServer
import rclpy
def main(args=None):
    rclpy.init(args=args)
    ftp_server = TransportFileServer("real")
    
    try:
        ftp_server.ftpserver.connect()
        ftp_server.ftpserver.serve()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect
        ftp_server.ftpserver.safe_kill()
        rclpy.shutdown()

if __name__ == '__main__':
    main()