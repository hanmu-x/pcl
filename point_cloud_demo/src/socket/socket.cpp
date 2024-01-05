#include "socket.h"

#include <cstring>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <thread>
#include <chrono>



#define MAX_BUFFER_SIZE  1024


bool Socket::tcpServer(std::string ip, int port)
{

	return false;
}

// 同步
bool Socket::tcpClientSync(std::string ip, int port)
{
    bool status = false;
    // WSAStartup() 函数初始化了Winsock库
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cout << "Failed to initialize winsock" << std::endl;
        return false;
    }
    while (true)
    {
        // 创建套接字
        SOCKET clientSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (clientSocket == INVALID_SOCKET) {
            std::cout << "Failed to create socket" << std::endl;
            WSACleanup();
            return false;
        }

        // 设置服务器信息
        sockaddr_in serverAddress;
        serverAddress.sin_family = AF_INET;
        serverAddress.sin_port = htons(port);
        if (inet_pton(AF_INET, ip.c_str(), &(serverAddress.sin_addr)) <= 0) {
            std::cout << "Invalid address/Address not supported" << std::endl;
            closesocket(clientSocket);
            WSACleanup();
            return false;
        }

        while (true) 
        {
            // 连接到服务器
            int is_con = connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress));
            if (is_con == SOCKET_ERROR) 
            {
                std::cout << "Connection failed" << std::endl;
                Sleep(5000); // 等待5秒后重新连接
                continue;
            }
            while (true)
            {
                char buffer[1024];
                memset(buffer, 0, sizeof(buffer));

                // 接收数据 (都是 <0 出错 =0 连接关闭 >0 接收到数据大小)
                size_t is_rec = recv(clientSocket, buffer, sizeof(buffer), 0);
                if ((int)is_rec == 0)
                {
                    std::cout << "Server disconnection and reconnection " << std::endl;
                    break;
                }
                else if((int)is_rec < 0)
                {
                    std::cout << "Failed to receive data" << std::endl;
                    continue;
                }
                else
                {
                    // 输出接收到的数据
                    std::cout << "Received data from server: " << buffer << std::endl;
                    Sleep(100); // 等待100毫秒后重新连接
                }
            }
            Sleep(100); // 跳出循环关闭socket 然后从新建立socket从新连接
            break;
        }

        closesocket(clientSocket);
        Sleep(3000); // 等待100毫秒后重新连接

    }

    WSACleanup();
    return true;
}


bool g_isRunning = true;

void DataReceiver(SOCKET clientSocket)
{
    char buffer[1024];
    while (g_isRunning)
    {
        memset(buffer, 0, sizeof(buffer));
        int is_rec = recv(clientSocket, buffer, sizeof(buffer), 0);
        if (is_rec == 0)
        {
            std::cout << "Server disconnection and reconnection" << std::endl;
            break;
        }
        else if (is_rec < 0)
        {
            std::cout << "Failed to receive data" << std::endl;
            break;
        }
        else
        {
            std::cout << "Received data from server: " << buffer << std::endl;
            // 处理接收到的数据
        }
    }
    closesocket(clientSocket);
    return;
}


bool Socket::tcpClientAsyn(std::string ip, int port)
{
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        std::cout << "Failed to initialize winsock" << std::endl;
        return false;
    }
    while (true)
    {
        // 创建套接字
        SOCKET clientSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (clientSocket == INVALID_SOCKET)
        {
            std::cout << "Failed to create socket" << std::endl;
            WSACleanup();
            return false;
        }

        // 设置服务器信息
        sockaddr_in serverAddress;
        serverAddress.sin_family = AF_INET;
        serverAddress.sin_port = htons(port);
        if (inet_pton(AF_INET, ip.c_str(), &(serverAddress.sin_addr)) <= 0)
        {
            std::cout << "Invalid address/Address not supported" << std::endl;
            closesocket(clientSocket);
            WSACleanup();
            return false;
        }

        while (true)
        {
            // 连接到服务器
            int is_con = connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress));
            if (is_con == SOCKET_ERROR)
            {
                std::cout << "Connection failed" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 等待1秒后重新连接
                break;
            }
            else
            {
                std::cout << "Connected to server" << std::endl;
                std::thread receiverThread(DataReceiver, clientSocket);
                receiverThread.detach();

                int n = 0;
                while (true)
                {
                    std::cout << n++ << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 等待1秒后重新连接

                    // 检查连接状态
                    int error = 0;
                    int errorLen = sizeof(error);
                    int ret = getsockopt(clientSocket, SOL_SOCKET, SO_ERROR, (char*)&error, &errorLen);
                    if (ret == SOCKET_ERROR || error != 0)
                    {
                        break;
                    }
                }
            }
        }

        closesocket(clientSocket);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 等待1秒后重新连接
    }

    WSACleanup();
    return true;
}






bool Socket::udpServer(std::string ip, int port)
{

	return false;
}

// 同步
bool Socket::updClientSync(std::string ip, int port)
{

	return false;
}

// 异步
bool Socket::updClientAsyn(std::string ip, int port)
{

	return false;
}




//Socket::Socket()
//{
//}
//
//Socket::~Socket()
//{
//}