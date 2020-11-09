#include <zmq.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>

//  Receive ZeroMQ string from socket and convert into C string
//  Chops string at 255 chars, if it's longer
static char *
s_recv (void *socket) {
    char buffer [256];
    int size = zmq_recv (socket, buffer, 255, 0);
    if (size == -1)
        return NULL;
    if (size > 255)
        size = 255;
    buffer [size] = '\0';
    /* use strndup(buffer, sizeof(buffer)-1) in *nix */
    return strdup (buffer);
}

int main (void)
{
    //  Socket to talk to clients
    void *context = zmq_ctx_new ();
    void *responder = zmq_socket (context, ZMQ_REP);
    int rc = zmq_bind (responder, "tcp://*:25555");
    assert (rc == 0);

    int i = 1;

    while (1) {
        char buffer [10];
        // zmq_recv (responder, buffer, 10, 0);

        char *msg = s_recv (responder);

        // char *msg = s_recv (responder);
        // printf("%s\n",msg);
        printf("Incomming Message #%i: %s\n", i, msg);
        // sleep (1);
        zmq_send (responder, "World", 5, 0);

        i++;
    }
    return 0;
}