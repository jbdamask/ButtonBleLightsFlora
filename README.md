# ButtonBleLightsFlora
Code for Adafruit Flora Bluefruit LE with button. This code allows for the control of attached NeoPixels by either button push or Bluetooth Low Energy. Button push events are packaged into a payload and written to UART TX for use by listeners.

Also uses NeoPixel RBG[W]

## Hardware
Set board pin to “DATA” (it doesn’t really matter but it’s good to be consistent.
Wire MODE pad to D12 pad on your Flora board

##Software
Add BluefruitConfig.h file
```#define BLUEFRUIT_UART_MODE_PIN        12 ```

Use hardware serial in your .ino file

```Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN); ```

Set the MODE pin to HIGH (CMD mode) in the setup() method. This will allow the board to take instructions if needed (like renaming)

``` // Set Flora Bluefruit LE to CMD mode
digitalWrite(BLUEFRUIT_UART_MODE_PIN, HIGH); 
```

After initialization, set to DATA mode on connect:

```ble.setMode(BLUEFRUIT_MODE_DATA);  ```

## Demo
https://www.youtube.com/watch?v=Ffhj39ibOfI

## ToDo
* Eventually, clean up the code. As I've been writing while learning and experimenting, this is a freaking mess. Don't hate