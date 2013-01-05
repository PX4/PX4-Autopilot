/* A simple server in the internet domain using TCP
   The port number is passed as an argument */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

int hosttcp_init(int portno);
size_t hosttcp_read(int fd, char * buffer, size_t len);
size_t hosttcp_write(int fd, const char * buffer, size_t len);
void hosttcp_close(int fd);

int
hosttcp_init(int portno)
{
     int sockfd, newsockfd;
     socklen_t clilen;
     struct sockaddr_in serv_addr, cli_addr;
     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0)
     {
        printf("error opening tcp:%d\n",portno);
        return NULL;
     }

     bzero((char *) &serv_addr, sizeof(serv_addr));
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);

     printf("listening on tcp:%d\n", portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
     {
        printf("error binding socket\n");
        return NULL;
     }
     listen(sockfd,5);
     clilen = sizeof(cli_addr);
     newsockfd = accept(sockfd, 
                 (struct sockaddr *) &cli_addr, 
                 &clilen);
     close(sockfd);
     if (newsockfd < 0) {
        printf("error on accept");
        return NULL;
     }
     return newsockfd;
}

size_t
hosttcp_read(int fd, char * buffer, size_t len)
{
    size_t n = 0;
    bzero(buffer,len);
    n = recv(fd,buffer,len,0);
    if (n < 0) error("ERROR reading from socket");
    return n;
}

size_t
hosttcp_write(int fd, const char * buffer, size_t len)
{
    size_t n = 0;
    n = send(fd,buffer,len,0);
    if (n < 0) error("ERROR writing to socket");
    return n;
}

void
hosttcp_close(int fd)
{
     close(fd);
}
