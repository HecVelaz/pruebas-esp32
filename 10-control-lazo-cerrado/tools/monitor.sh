#!/bin/sh
# Monitor serie de la WROOM sin tocar DTR/RTS (el de PlatformIO metía basura al abrir). Salir con Ctrl+]
exec python3 -m serial.tools.miniterm --eol LF --echo --dtr 0 --rts 0 \
  /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 115200
