#include <cstddef>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>

#include "rtc/rtc.hpp"
// #include <nlohmann/json.hpp>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

using Socket = int;
// using json = nlohmann::json;

const int BUFFER_SIZE = 2048;

std::shared_ptr<rtc::Track> makeTrack(int &sock, const std::shared_ptr<rtc::PeerConnection> &pc)
{
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    addr.sin_port = htons(6000);

    if (bind(sock, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        throw std::runtime_error("Failed to bind socket");
    }

    int rcvBufSize = 212992;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char *>(&rcvBufSize),
               sizeof(rcvBufSize));

    const rtc::SSRC ssrc = 42;
    rtc::Description::Video media("video", rtc::Description::Direction::SendOnly);
    media.addH264Codec(96); // Must match the payload type of the external h264 RTP stream
    media.addSSRC(ssrc, "video-send");

    return pc->addTrack(media);
}

int main()
{
    rtc::InitLogger(rtc::LogLevel::Debug);

    auto ws = std::make_shared<rtc::WebSocket>();
    auto pc = std::make_shared<rtc::PeerConnection>();

    int sock;
    auto track = makeTrack(sock, pc);
    std::cout << "Created track" << std::endl;

    ws->onOpen(
        [&]()
        {
            std::cout << "WebSocket connected, signaling ready" << std::endl;
        });

    ws->onClosed(
        []()
        {
            std::cout << "WebSocket closed" << std::endl;
        });

    ws->onError(
        [](const std::string &error)
        {
            std::cout << "WebSocket failed: " << error << std::endl;
        });

    pc->onStateChange(
        [](rtc::PeerConnection::State state)
        {
            std::cout << "State: " << state << std::endl;
        });

    pc->onGatheringStateChange(
        [](rtc::PeerConnection::GatheringState state)
        {
            std::cout << "Gathering state: " << state << std::endl;
        });

    // pc->onLocalDescription(
    //     [&](const rtc::Description &description)
    //     {
    //         json message = {"offer", {{"type", description.typeString()}, {"sdp", std::string(description)}}};

    //         ws->send(message.dump());
    //     });

    pc->onLocalCandidate(
        [&](const rtc::Candidate &candidate)
        {
            json message = {"candidate", candidate};

            ws->send(message.dump());
        });

    ws->onMessage(
        [&](std::variant<rtc::binary, std::string> data)
        {
            if (!std::holds_alternative<std::string>(data))
            {
                return;
            }

            json message = json::parse(std::get<std::string>(data));

            // Will either be "offer" or "candidate"
            if (message.contains("offer"))
            {
                std::cout << "Received offer" << std::endl;

                rtc::Description offer(
                    message["offer"]["sdp"].get<std::string>(),
                    message["offer"]["type"].get<std::string>());

                rtc::Description offer()

                pc->setRemoteDescription(offer);

                // Receive from UDP
                char buffer[BUFFER_SIZE];
                int len;
                while ((len = recv(sock, buffer, BUFFER_SIZE, 0)) >= 0)
                {
                    if (len < sizeof(rtc::RtpHeader) || !track->isOpen())
                        continue;

                    auto rtp = reinterpret_cast<rtc::RtpHeader *>(buffer);
                    rtp->setSsrc(42);

                    track->send(reinterpret_cast<const std::byte *>(buffer), len);
                }
            }
            else if (message.contains("candidate"))
            {
                std::cout << "Received candidate" << std::endl;

                pc->addRemoteCandidate(rtc::Candidate(message["candidate"].get<std::string>()));
            }
            else
            {
                std::cout << "Unknown message type" << std::endl;
            }
        });

    ws->open("ws://127.0.0.1:8000");

    return 0;
}