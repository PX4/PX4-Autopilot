# W tym pliku zawarta jest dokumentacja każdego z dodatkowych modułów zaimplementowanych na własne potrzeby

## Preflight Calibration
Uruchamiany przed startem platformy - służy do zweryfikowania poprawności działania krytycznych komponentów systemu.

Moduł ma charakter blokujący, to znaczy dopiero po zaliczeniu pomyślnie wszystkich testów należących do preflight możliwe jest uzbrojenie silników i start.

### PreflightCalibrationControl message
uint64 timestamp
uint8 action # 0 - STOP, 1 - START

### Hardware Setup
Aby skonfigurować stanowisko testowe do modułu Preflight należy podłączyć wtyczkę z portu TELEM2 OrangeCube do GPIO14 i GPIO15 (UART1).

Do tego trzeba zaktualizować firmware w Orange Cube. Należy w tym repozytorium zpullować najnowsze zmiany i następnie zbudować firmware na odpowiedni target:

```
make cubepilot_cubeorangeplus
```

Następnie, gdy budowanie przebiegnie poprawnie należy wgrać kod na hardware komendą:

```
make cubepilot_cubeorangeplus upload
```

Teraz żeby przetestować moduł należy uruchomić moduł. Najpierw sprawdzamy w konsoli nuttx czy moduł jest obecny w firmware:

```
prefight_calibration status
```

Teraz trzeba wysłać odpowiednią wiadomość uORB z klienta XRCE. W tym celu należy stworzyć odpowiednią 'misję testową' w któej wyślemy odpowiednią wiadomość uORB 'PreflighCalibrationControl z wartością action = 1.

To powinno striggerować moduł do kalibracji i automatyczną kalibracje serwomechanizmów. Można też to zrobić poprzez (w przyszłości tak powinno być) wpisanie w konsoli

```
prefight_calibration start
```


### Przebieg czynności:
1. Status przyrządów pokładowych - imu, GPS, pitot
2. Kalibracja akcelerometru, giroskopu
3. Status połączenia z CC, skrypt testujący łączność
4. Sprawdzenie łączności radiowej i telemetrii
5. Manualna weryfikacja aktuatorów
6. Automatyczny test mechanizmu obrotu
7. Status transmisji wideo

Ad. 1 - otrzymanie wiadomości statusowej od każdego z driverów
Ad. 3 - testy Microsoft x
Ad. 6 - Na podstawie parametrów i danych telemetrycznych należy wykonać sekwencje badającą cały zakres ruchu mechanizmu. Do tego weryfikacja danych o mocy i natężeniu prądu.

### Wiadomość MAVLink
PREFLIGHT_CALIBRATION_STATUS

Wiadomość MAVLink przechowuje status kalibracji.

uint8_t servo_bitmask
float servo_positions[16]
float min_values[16]
float max_values[16]
bool calibration_succesfull


