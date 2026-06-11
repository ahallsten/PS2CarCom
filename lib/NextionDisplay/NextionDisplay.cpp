#include "NextionDisplay.h"

NextionDisplay::NextionDisplay(Stream &uart) : _uart(uart) {}

void NextionDisplay::endCommand() {
#if NEXTION_ENABLED
  _uart.write(0xFF);
  _uart.write(0xFF);
  _uart.write(0xFF);
#endif
}

void NextionDisplay::writeRaw(const char *s) {
#if NEXTION_ENABLED
  _uart.write(reinterpret_cast<const uint8_t *>(s), strlen(s));
#else
  (void)s;
#endif
}

void NextionDisplay::begin() {
#if NEXTION_ENABLED
  // A leading terminator clears any partial command in the panel's RX buffer.
  endCommand();
  showLogPage();
#endif
}

void NextionDisplay::setText(const char *component, const char *value) {
#if NEXTION_ENABLED
  writeRaw(component);
  writeRaw(".txt=\"");
  writeRaw(value);
  writeRaw("\"");
  endCommand();
#else
  (void)component;
  (void)value;
#endif
}

void NextionDisplay::setNumber(const char *component, long value) {
#if NEXTION_ENABLED
  char num[12];
  ltoa(value, num, 10);
  writeRaw(component);
  writeRaw(".val=");
  writeRaw(num);
  endCommand();
#else
  (void)component;
  (void)value;
#endif
}

void NextionDisplay::setVisible(const char *component, bool visible) {
#if NEXTION_ENABLED
  writeRaw("vis ");
  writeRaw(component);
  writeRaw(visible ? ",1" : ",0");
  endCommand();
#else
  (void)component;
  (void)visible;
#endif
}

void NextionDisplay::setPage(const char *pageName) {
#if NEXTION_ENABLED
  writeRaw("page ");
  writeRaw(pageName);
  endCommand();
#else
  (void)pageName;
#endif
}

void NextionDisplay::appendLogLine(const char *text) {
  setText(NEX_LOG_TEXT, text);
}

void NextionDisplay::showLogPage() {
  _page = Page::Log;
  setPage(NEX_PAGE_LOG);
}

void NextionDisplay::showDashboardPage() {
  _page = Page::Dash;
  setPage(NEX_PAGE_DASH);
}

void NextionDisplay::togglePage() {
  if (_page == Page::Log) {
    showDashboardPage();
  } else {
    showLogPage();
  }
}
