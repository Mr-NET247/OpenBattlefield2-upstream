#include <array>
#include <chrono>
#include <thread>

#if defined(_WIN32)
#include <winsock2.h>
#else
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <SDL3/SDL.h>

#include "check.h"
#include "obf2/app/match_voice.h"
#include "obf2/net/retail_voice.h"
#include "obf2/session/remote_world.h"

namespace {
using namespace obf2;
namespace bf2 = net::bf2;
using namespace std::chrono_literals;

#if defined(_WIN32)
using Socket = SOCKET;
using AddressLength = int;
constexpr Socket kInvalidSocket = INVALID_SOCKET;
#else
using Socket = int;
using AddressLength = socklen_t;
constexpr Socket kInvalidSocket = -1;
#endif

class Relay {
 public:
  Relay() {
#if defined(_WIN32)
    WSADATA data{};
    started_ = ::WSAStartup(MAKEWORD(2, 2), &data) == 0;
    if (!started_) return;
#endif
    socket = ::socket(AF_INET, SOCK_DGRAM, 0);
  }

  ~Relay() {
#if defined(_WIN32)
    if (socket != kInvalidSocket) ::closesocket(socket);
    if (started_) ::WSACleanup();
#else
    if (socket != kInvalidSocket) ::close(socket);
#endif
  }

  Relay(const Relay&) = delete;
  Relay& operator=(const Relay&) = delete;

  int receive(std::span<std::byte> packet, sockaddr_in& from, int timeoutMs) const {
    fd_set ready;
    FD_ZERO(&ready);
    FD_SET(socket, &ready);
    timeval timeout{timeoutMs / 1000, (timeoutMs % 1000) * 1000};
#if defined(_WIN32)
    const int descriptorCount = 0;  // Winsock ignores this argument.
#else
    const int descriptorCount = socket + 1;
#endif
    if (::select(descriptorCount, &ready, nullptr, nullptr, &timeout) != 1) return -1;
    AddressLength length = sizeof(from);
    return static_cast<int>(::recvfrom(socket, reinterpret_cast<char*>(packet.data()),
                                      static_cast<int>(packet.size()), 0,
                                      reinterpret_cast<sockaddr*>(&from), &length));
  }

  Socket socket = kInvalidSocket;

 private:
#if defined(_WIN32)
  bool started_ = false;
#endif
};

void setupClient(session::RemoteWorld& match, unsigned player, std::uint16_t port) {
  match.socket = net::UdpSocket::connect("127.0.0.1", port);
  match.databaseSentAt = std::chrono::steady_clock::now();
  match.serverMatchOptions = bf2::ServerMatchOptions{false, false, false, true, 1, port, 0, 3};
  match.voiceSession = 400 + player;
  match.world.setOwnConnection(player);
  bf2::Event first, second, role;
  first.player = bf2::CreatePlayer{1, 0, 1, 101, false, "Voice A"};
  second.player = bf2::CreatePlayer{2, 0, 2, 102, false, "Voice B"};
  role.commanderAssignment = bf2::Event::CommanderAssignment{player, static_cast<int>(player)};
  match.world.feedEvents({first, second, role});
}

void testVoiceRelay() {
  CHECK(SDL_SetHint(SDL_HINT_AUDIO_DRIVER, "dummy"));
  const Relay relay;
  CHECK(relay.socket != kInvalidSocket);
  if (relay.socket == kInvalidSocket) return;
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  AddressLength addressLength = sizeof(address);
  if (::bind(relay.socket, reinterpret_cast<sockaddr*>(&address), sizeof(address)) ||
      ::getsockname(relay.socket, reinterpret_cast<sockaddr*>(&address), &addressLength)) {
    CHECK(false);
    return;
  }

  {
    FileSystem files;
    session::RemoteWorld first({}, files), second({}, files);
    setupClient(first, 1, ntohs(address.sin_port));
    setupClient(second, 2, ntohs(address.sin_port));
    app::MatchVoice voiceA, voiceB;
    voiceA.update(first, false, false, true);
    voiceB.update(second, false, false, true);
    CHECK(voiceA.available() && voiceB.available());

    std::array<sockaddr_in, 2> peers{};
    std::array<std::byte, 1024> packet{};
    for (unsigned i = 0; i < peers.size(); ++i) {
      const int size = relay.receive(packet, peers[i], 100);
      CHECK(size == 3 && packet[0] == std::byte(401 + i - 256) &&
            packet[1] == std::byte{1} && packet[2] == std::byte{6});
    }

    std::array<bool, 2> relayed{};
    auto codec = bf2::RetailVoiceCodec::create(3);
    CHECK(codec);
    for (int attempt = 0; codec && attempt < 50 && !(relayed[0] && relayed[1]); ++attempt) {
      voiceA.update(first, false, true, true);
      voiceB.update(second, false, true, true);
      for (int budget = 0; budget < 16; ++budget) {
        sockaddr_in from{};
        const int size = relay.receive(packet, from, 0);
        if (size < 0) break;
        const auto frameBytes = codec->frameBytes();
        CHECK_EQ(size, static_cast<int>(frameBytes + 5));
        if (size != static_cast<int>(frameBytes + 5)) continue;
        const unsigned sender = from.sin_port == peers[0].sin_port ? 0 : 1;
        CHECK_EQ(from.sin_port, peers[sender].sin_port);
        CHECK(packet[frameBytes + 2] == std::byte(401 + sender - 256) &&
              packet[frameBytes + 3] == std::byte{1} && packet[frameBytes + 4] == std::byte{1});
        std::array<std::int16_t, bf2::kVoiceFrameSamples> pcm{};
        CHECK(codec->decode(std::span(packet).first(frameBytes), pcm));
        packet[frameBytes + 2] = std::byte(sender + 1);
        packet[frameBytes + 3] = std::byte{0};
        const auto& destination = peers[1 - sender];
        const auto relayBytes = static_cast<int>(frameBytes + 4);
        const auto sent = ::sendto(relay.socket, reinterpret_cast<const char*>(packet.data()),
                                   relayBytes, 0, reinterpret_cast<const sockaddr*>(&destination),
                                   sizeof(destination));
        CHECK_EQ(sent, relayBytes);
        relayed[sender] = true;
      }
      std::this_thread::sleep_for(20ms);
    }
    CHECK(relayed[0] && relayed[1]);
    voiceA.update(first, false, false, true);
    voiceB.update(second, false, false, true);
    voiceA.stop(first);
    voiceB.stop(second);
  }
  SDL_Quit();
}

}  // namespace

TEST_MAIN({ testVoiceRelay(); });
