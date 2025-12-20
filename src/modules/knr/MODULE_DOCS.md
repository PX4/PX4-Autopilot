# W tym pliku zawarta jest dokumentacja każdego z dodatkowych modułów zaimplementowanych na własne potrzeby

## Preflight Calibratio

Uruchamiany przed startem platformy - służy do zweryfikowania poprawności działania krytycznych komponentów systemu.

Moduł ma charakter blokujący, to znaczy dopiero po zaliczeniu pomyślnie wszystkich testów należących do preflight możliwe jest uzbrojenie silników i start.

### Przebieg czynności:

1. Status przyrządów pokładowych - imu, GPS, pitot
2. Kalibracja IMU, giroskopu
3. Status połączenia z CC, skrypt testujący łączność
4. Sprawdzenie łączności radiowej i telemetrii
5. Manualna weryfikacja aktuatorów
6. Automatyczny test mechanizmu obrotu
7. Status transmisji wideo

Ad. 1 - otrzymanie wiadomości statusowej od każdego z driverów
Ad. 3 - testy Microsoft x
Ad. 6 - Na podstawie parametrów i danych telemetrycznych należy wykonać sekwencje badającą cały zakres ruchu mechanizmu. Do tego weryfikacja danych o mocy i natężeniu prądu.
