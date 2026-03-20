import socket
import concurrent.futures

# Your G14 laptop IP on the lab WiFi
MY_IP = "10.155.234.162" #chgange to ur ip address
SUBNET = "10.155.234."

def check_ssh_port(ip):
    # Exclude your own laptop
    if ip == MY_IP:
        return None
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.5) # Fast timeout so it does not hang
    
    try:
        # Check if port 22 is accepting connections
        if sock.connect_ex((ip, 22)) == 0:
            return ip
    except Exception:
        pass
    finally:
        sock.close()
        
    return None

def main():
    print(f"Scanning subnet {SUBNET}x for open SSH ports...")
    ips = [f"{SUBNET}{i}" for i in range(1, 255)]
    
    # Run multiple checks at the same time to speed it up
    with concurrent.futures.ThreadPoolExecutor(max_workers=50) as executor:
        results = executor.map(check_ssh_port, ips)
        
    print("Results:")
    for ip in results:
        if ip:
            print(f"Works: {ip}")

if __name__ == "__main__":
    main()