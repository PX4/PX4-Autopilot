# SubscribeBigHelloWorld example

This example will show how to receive big data (up to 64 kB) to the DDS World creating a client publisher.
In order to compile this example, the following profiles should be enabled:

- `UCLIENT_PROFILE_UDP`

## Usage
1. Run an agent in a certain port, for example, *2018*: `MicroXRCEAgent udp4 -p 2018`.
2. Run the *PublisherBigHelloWorld* example or some publisher that can send the *BigHelloWorld* topic.
3. Run the *SubscriberBigHelloWorld* example.
   The example expects first and second argument to be IP address and port where the Micro XRCE-DDS Agent is running. It can also be parameterized with the amount of times that the topic will be listened.
   If no number is given, the publisher will publish indefinitely.

## Considerations

- Notice that `BUFFER_SIZE` shall be big enough to store the whole message.
- Notice that `STREAM_HISTORY` shall power of two.

