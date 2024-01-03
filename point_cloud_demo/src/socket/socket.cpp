#include "socket.h"

#include <cstring>
#include <winsock2.h>
#include <ws2tcpip.h>

bool Socket::tcpServer(std::string ip, int port)
{

	return false;
}

// ͬ��
bool Socket::tcpClientSync(std::string ip, int port)
{
    bool status = false;
    // WSAStartup() ������ʼ����Winsock��
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cout << "Failed to initialize winsock" << std::endl;
        return false;
    }
    while (true)
    {
        // �����׽���
        SOCKET clientSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (clientSocket == INVALID_SOCKET) {
            std::cout << "Failed to create socket" << std::endl;
            WSACleanup();
            return false;
        }

        // ���÷�������Ϣ
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
            // ���ӵ�������
            if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == SOCKET_ERROR) {
                std::cout << "Connection failed" << std::endl;
                Sleep(5000); // �ȴ�5�����������
                continue;
            }
            while (true)
            {
                char buffer[1024];
                memset(buffer, 0, sizeof(buffer));

                // ��������
                if (recv(clientSocket, buffer, sizeof(buffer), 0) == SOCKET_ERROR) 
                {
                    std::cout << "Failed to receive data" << std::endl;
                    continue;
                }
                // ������յ�������
                std::cout << "Received data from server: " << buffer << std::endl;
                Sleep(100); // �ȴ�100�������������
            }
            // �ر��׽���
            Sleep(100); // �ȴ�100�������������
            continue;
        }

        closesocket(clientSocket);
        Sleep(3000); // �ȴ�100�������������

    }

    WSACleanup();
    return true;
}

static const int MAX_BUFFER_SIZE = 1024;

static void CALLBACK asyncSocketCallback(DWORD error, DWORD transferredBytes, LPWSAOVERLAPPED overlapped, DWORD flags)
{
    if (error == 0) {
        // �첽�����ɹ����
        std::cout << "Async operation completed successfully" << std::endl;
    }
    else {
        // �첽��������
        std::cerr << "Async operation failed with error: " << error << std::endl;
    }
}

// �첽
bool Socket::tcpClientAsyn(std::string ip, int port)
{

    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cout << "Failed to initialize winsock" << std::endl;
        return false;
    }

    SOCKET clientSocket;
    sockaddr_in serverAddress;

    // �����׽���
    clientSocket = WSASocket(AF_INET, SOCK_STREAM, 0, NULL, 0, WSA_FLAG_OVERLAPPED);
    if (clientSocket == INVALID_SOCKET) {
        std::cout << "Failed to create socket" << std::endl;
        WSACleanup();
        return false;
    }

    // ���÷�������Ϣ
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &(serverAddress.sin_addr)) <= 0) {
        std::cout << "Invalid address/Address not supported" << std::endl;
        closesocket(clientSocket);
        WSACleanup();
        return false;
    }

    // ���ӵ�������
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == SOCKET_ERROR) {
        if (WSAGetLastError() != WSAEWOULDBLOCK) {
            std::cout << "Connection failed" << std::endl;
            closesocket(clientSocket);
            WSACleanup();
            return false;
        }
    }

    // �첽��������
    WSAOVERLAPPED overlapped;
    ZeroMemory(&overlapped, sizeof(WSAOVERLAPPED));
    char buffer[MAX_BUFFER_SIZE];
    WSABUF dataBuffer;
    dataBuffer.len = MAX_BUFFER_SIZE;
    dataBuffer.buf = buffer;
    overlapped.hEvent = buffer;
    DWORD receivedBytes;
    DWORD flags = 0;
    if (WSARecv(clientSocket, &dataBuffer, 1, &receivedBytes, &flags, &overlapped, asyncSocketCallback) == SOCKET_ERROR) {
        if (WSAGetLastError() != WSA_IO_PENDING) {
            std::cout << "Failed to initiate async receive" << std::endl;
            closesocket(clientSocket);
            WSACleanup();
            return false;
        }
    }

    // ��������
    std::string message = "Hello, server!";
    if (send(clientSocket, message.c_str(), message.size(), 0) == SOCKET_ERROR) {
        if (WSAGetLastError() != WSAEWOULDBLOCK) {
            std::cout << "Failed to send data" << std::endl;
            closesocket(clientSocket);
            WSACleanup();
            return false;
        }
    }

    // �ȴ��첽�������
    DWORD result;
    if (WSAGetOverlappedResult(clientSocket, &overlapped, &receivedBytes, TRUE, &flags) == FALSE) {
        std::cout << "Failed to get overlapped result" << std::endl;
    }

    // ������յ�������
    std::cout << "Received data from server: " << buffer << std::endl;

    // �ر��׽���
    closesocket(clientSocket);
    WSACleanup();

    return true;
}


bool Socket::udpServer(std::string ip, int port)
{

	return false;
}

// ͬ��
bool Socket::updClientSync(std::string ip, int port)
{

	return false;
}

// �첽
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