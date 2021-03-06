#include "mbed.h"
#include "EthernetInterface.h"
#include "DualMC33926MotorShield.h"

// Network interface
EthernetInterface net;

// Striker motors
DualMC33926MotorShield Dampener(PA_7, PB_8);

void connect_http() {
  // Show the network address
  const char *ip = net.get_ip_address();
  const char *mac = net.get_mac_address();
  printf("IP address is: %s\n", ip ? ip : "No IP");
  printf("MAC address is: %s\n", mac ? mac : "No MAC");

  // Open a socket on the network interface, and create a TCP connection to mbed.org
  TCPSocket socket;
  socket.open(&net);
  socket.connect("developer.mbed.org", 80);

  // Send a simple http request
  char sbuffer[] = "GET / HTTP/1.1\r\nHost: developer.mbed.org\r\n\r\n";
  int scount = socket.send(sbuffer, sizeof sbuffer);
  printf("sent %d [%.*s]\n", scount, strstr(sbuffer, "\r\n")-sbuffer, sbuffer);

  // Recieve a simple http response and print out the response line
  char rbuffer[64];
  int rcount = socket.recv(rbuffer, sizeof rbuffer);
  printf("recv %d [%.*s]\n", rcount, strstr(rbuffer, "\r\n")-rbuffer, rbuffer);

  // Close the socket to return its memory and bring down the network interface
  socket.close();
}


int main() {
  printf("Ethernet socket test\n");
  net.connect();
  Thread t1;
  t1.start(callback(connect_http));
  Thread::wait(osWaitForever);
}
