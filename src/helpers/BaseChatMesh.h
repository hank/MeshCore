#pragma once

#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>
#include <helpers/AdvertDataHelpers.h>

#define MAX_TEXT_LEN    (10*CIPHER_BLOCK_SIZE)  // must be LESS than (MAX_PACKET_PAYLOAD - 4 - CIPHER_MAC_SIZE - 1)

struct ContactInfo {
  mesh::Identity id;
  char name[32];
  uint8_t type;   // on of ADV_TYPE_*
  uint8_t flags;
  int8_t out_path_len;
  uint8_t out_path[MAX_PATH_SIZE];
  uint32_t last_advert_timestamp;
  uint8_t shared_secret[PUB_KEY_SIZE];
};

#define MAX_SEARCH_RESULTS   8

#define MSG_SEND_FAILED       0
#define MSG_SEND_SENT_FLOOD   1
#define MSG_SEND_SENT_DIRECT  2

class ContactVisitor {
public:
  virtual void onContactVisit(const ContactInfo& contact) = 0;
};

class BaseChatMesh;

class ContactsIterator {
  int next_idx = 0;
public:
  bool hasNext(const BaseChatMesh* mesh, ContactInfo& dest);
};

/**
 *  \brief  abstract Mesh class for common 'chat' client
 */
class BaseChatMesh : public mesh::Mesh {

  friend class ContactsIterator;

  ContactInfo contacts[MAX_CONTACTS];
  int num_contacts;
  int sort_array[MAX_CONTACTS];
  int matching_peer_indexes[MAX_SEARCH_RESULTS];
  unsigned long txt_send_timeout;

  mesh::Packet* composeMsgPacket(const ContactInfo& recipient, uint8_t attempt, const char *text, uint32_t& expected_ack);

protected:
  BaseChatMesh(mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::PacketManager& mgr, mesh::MeshTables& tables)
      : mesh::Mesh(radio, ms, rng, rtc, mgr, tables)
  { 
    num_contacts = 0;
    txt_send_timeout = 0;
  }

  // 'UI' concepts, for sub-classes to implement
  virtual void onDiscoveredContact(ContactInfo& contact, bool is_new) = 0;
  virtual bool processAck(const uint8_t *data) = 0;
  virtual void onContactPathUpdated(const ContactInfo& contact) = 0;
  virtual void onMessageRecv(const ContactInfo& contact, bool was_flood, uint32_t sender_timestamp, const char *text) = 0;
  virtual uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const = 0;
  virtual uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const = 0;
  virtual void onSendTimeout() = 0;

  // Mesh overrides
  void onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id, uint32_t timestamp, const uint8_t* app_data, size_t app_data_len) override;
  int searchPeersByHash(const uint8_t* hash) override;
  void getPeerSharedSecret(uint8_t* dest_secret, int peer_idx) override;
  void onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx, const uint8_t* secret, uint8_t* data, size_t len) override;
  bool onPeerPathRecv(mesh::Packet* packet, int sender_idx, const uint8_t* secret, uint8_t* path, uint8_t path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) override;
  void onAckRecv(mesh::Packet* packet, uint32_t ack_crc) override;

public:
  mesh::Packet* createSelfAdvert(const char* name);
  int  sendMessage(const ContactInfo& recipient, uint8_t attempt, const char* text, uint32_t& expected_ack);
  void resetPathTo(ContactInfo& recipient);
  void scanRecentContacts(int last_n, ContactVisitor* visitor);
  ContactInfo* searchContactsByPrefix(const char* name_prefix);
  bool  addContact(const ContactInfo& contact);

  void loop();
};
