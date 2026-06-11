#ifndef NEXTION_DISPLAY_H
#define NEXTION_DISPLAY_H

#include <Arduino.h>
#include "NextionConfig.h"

/**
 * @brief Minimal non-blocking Nextion HMI backend over a generic Stream.
 *
 * The display is driven through any Stream (intended to be a hardware UART such
 * as Serial1 on the Feather 32u4) so this class compiles and runs whether or
 * not a physical panel is attached. Every command is terminated with the
 * Nextion-required three 0xFF bytes. Commands are written piecewise to the
 * stream to avoid large temporary buffers on the ATmega32u4.
 *
 * When NEXTION_ENABLED is 0 every method is a no-op (page state is still
 * tracked) so callers need no conditional compilation of their own.
 */
class NextionDisplay {
public:
  /** @brief Backend-tracked current page. */
  enum class Page : uint8_t { Log = 0, Dash = 1 };

  /**
   * @brief Construct around an already-configured stream.
   *
   * The caller is responsible for begin()-ing the underlying UART (e.g.
   * Serial1.begin(NEXTION_BAUD)).
   *
   * @param uart Stream used to talk to the Nextion panel.
   */
  explicit NextionDisplay(Stream &uart);

  /** @brief Send initial commands and select the log page. */
  void begin();

  /**
   * @brief Set a text component's .txt value.
   *
   * @param component Fully qualified component name, e.g. "dash.tLink".
   * @param value Text to display. Avoid embedded double-quote characters.
   */
  void setText(const char *component, const char *value);

  /**
   * @brief Set a numeric component's .val.
   *
   * @param component Fully qualified component name.
   * @param value Integer value to display.
   */
  void setNumber(const char *component, long value);

  /**
   * @brief Show or hide a component (Nextion `vis`).
   *
   * @param component Component name.
   * @param visible true to show, false to hide.
   */
  void setVisible(const char *component, bool visible);

  /**
   * @brief Switch the Nextion to a page by name (`page <name>`).
   *
   * @param pageName Page name as defined in the editor.
   */
  void setPage(const char *pageName);

  /**
   * @brief Replace the log page text component with the provided text.
   *
   * @param text Text (may contain "\r\n" line breaks) to show in NEX_LOG_TEXT.
   */
  void appendLogLine(const char *text);

  /** @brief Select the log page. */
  void showLogPage();

  /** @brief Select the dashboard page. */
  void showDashboardPage();

  /** @brief Toggle between log and dashboard pages. */
  void togglePage();

  /** @brief Current backend-tracked page. */
  Page currentPage() const { return _page; }

private:
  void endCommand();              ///< Emit the 0xFF 0xFF 0xFF terminator.
  void writeRaw(const char *s);   ///< Write a C string with no terminator.

  Stream &_uart;
  Page _page = Page::Log;
};

#endif
