# PM5000S

Linux 운영체제에서 PM5000S 미세먼지 센서와 UART 통신을 위한 SW

PM5000S와 CLI 대화형 통신 프로그램인 `pm5000s`와 측정 기록을 CSV파일로 계속해서 남기는 Service용 데몬 프로그램 `pm5000sd`로 이루어져 있습니다.

## Build Prerequisite

- [CMake] (>=3.18)
- C++ Compiler supporting C++17

## How to Build

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

빌드 후 설치까지 하러면 다음 명령어 수행합니다.

```sh
sudo cmake --install build
```

## How to Install from Packaged Shell Script

다음은 배포된 `pm5000s-\<version\>-Linux.sh` 에서 설치하는 방법입니다.

```sh
sudo ./pm5000s-<version>-Linux.sh --prefix=/usr/local --exclude-subdir --skip-license
```

## How to Use

다음은 라즈베리 파이에서 해당 SW 사용법입니다.

1. 라즈베리파이 GPIO 시리얼 포트 설정을 해줍니다.

    라즈베리 파이 GPIO 시리얼 포트 설정하는 방법은 [여기](https://m.blog.naver.com/emperonics/222039301356) 블로그에 자세히 설명되어 있습니다.  
    관련한 라즈베리 파이 공식 문서는 [여기](https://www.raspberrypi.org/documentation/configuration/uart.md)에서 확인할 수 있습니다.

2. `pm5000s` 명령어를 통해 PM5000S 센서의 Calibration Coefficient 설정 및 동작을 확인합니다.

    통신할 센서가 연결된 디바이스를 인자로 `pm5000s`를 실행합니다.
    예를 들어, `/dev/ttyAMA1` 에 연결된 센서와 통신하려면 다음처럼 실행합니다.

    ```sh
    pm5000s /det/ttyAMA1
    ```

    유효한 디바이스 경로를 인자로 실행하면 다음처럼 커맨드 별로 할당된 커맨드 번호가 나오고 해당되는 커맨드 번호를 입력하면 커맨드 실행 결과를 확인할 수 있습니다.

    ```sh
    Enter Command Number you want to execute

    1. Read Serial Number
    2. Read SW Version Number
    3. Read Particle Calibration Coefficient
    4. Set up Particle Calibration Coefficient
    5. Open Particle Measurement
    6. Close Particle Measurement
    7. Read Particle Measurement
    8. Read Device Path
    9. Exit
    ```

    예를 들어, 센서의 시리얼 넘버를 읽고 싶으면 **1** 을 입력하고 **Enter** 를 누릅니다.  
    또는, `Calibration Coefficient` 설정을 바꾸고 싶으면 **4** 를 입력하고 **Enter** 를 누른 뒤, 원하는 Coefficient 값(10~250)을 입력하고 다시 **Enter** 를 누릅니다.

    프로그램 종료를 하려면 **9** 를 입력하고 **Enter** 를 누르거나 **Ctrl + c** 를 입력합니다.

3. `systemctl` 명령어를 통해 `pm5000sd` 서비스를 시작 및 등록합니다.

    다음 명령어를 통해 `/dev/ttyAMA1` 에 연결된 센서에서 주기적으로 측정한 결과를 `/etc/pm5000s/logs`에 저장합니다.

    ```sh
    systemctl start pm5000s@/dev/ttyAMA1
    ```

    재부팅 후에도 해당 서비스가 자동으로 시작되도록 하려면 다음 명령어를 입력합니다.

    ```sh
    systemctl enable pm5000s@/dev/ttyAMA1
    ```

    서비스 동작 확인은 다음 명령어를 통해 확인할 수 있습니다.

    ```sh
    systemctl status pm5000s@/dev/ttyAMA1
    ```

    다른 디바이스에 대해서도 서비스 명의 `@` 뒤에 해당하는 디바이스 경로를 입력하여 서비스를 실행할 수 있습니다.

    해당 서비스의 측정 주기 및 재시도 횟수는 `/usr/local/etc/pm5000s/pm5000sd.config` 파일을 수정하면 됩니다.  
    `pm5000sd.config` 파일은 다음과 같이 생겼습니다.

    ```txt
    measure_interval_ms = 1000
    retry_limit = 10
    ```

    예를 들어, 측정 주기를 5초, 실패 시 최대 재시도 횟수를 5로 변경하려면 다음처럼 수정하면 됩니다.

    ```txt
    measure_interval_ms = 5000
    retry_limit = 5
    ```

    > `measure_interval_ms` 의 설정 가능 범위는 10 ~ 300000(10ms ~ 5min) 이고,  
    > `retry_limit`은  1 ~ 100 입니다.

[CMake]: https://cmake.org
