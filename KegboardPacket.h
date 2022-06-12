#include <inttypes.h>

class KegboardPacket {
  public:
   KegboardPacket();
   void SetType(int type) {m_type = type;}
   int GetType() {return m_type;}
   void AddTag(uint8_t tag, uint8_t buflen, const char *buf);

   bool ReadTag(uint8_t tagnum, uint8_t *value);
   int CopyTagData(uint8_t tagnum, void *dest);

   // Returns length of tag's payload, or -1 if not found.
   int FindTagLength(uint8_t tagnum);
   uint8_t* FindTag(uint8_t tagnum);

   void AppendBytes(const char *buf, int buflen);
   void Reset();
   bool IsReset();
   void Print();
   uint16_t GenCrc();
  private:
   int m_type;
   uint8_t m_len;
   uint8_t m_payload[KBSP_PAYLOAD_MAXLEN];
};
