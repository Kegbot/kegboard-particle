#define ONEWIRE_FAMILY_DS18B20  0x28
#define ONEWIRE_FAMILY_DS18S20  0x10
#define INVALID_TEMPERATURE_VALUE INT_MIN

class OneWire;

class DS1820Sensor {
  public:
   DS1820Sensor();
   bool Update(unsigned long clock);
   void PrintTemp(void);
   long GetTemp();
   void GetTempC(char* buf);
   void GetName(char* buf);
   bool Busy();
   bool Initialized();
   bool IsValid() { return m_temp_is_valid; }
   void Reset();
   int CompareId(uint8_t* other);
   void Initialize(OneWire* bus, uint8_t* addr);

   uint8_t m_addr[8];

  private:
   bool ResetAndSelect();
   bool StartConversion();
   bool FetchConversion();
   void Reset(uint8_t *addr);

  private:
   OneWire* m_bus;

   bool m_initialized;
   bool m_converting;
   unsigned long m_conversion_start_clock;
   int m_temp;
   bool m_temp_is_valid;
};
