# Mastering Zephyr Driver Development

This repository contains the code for the "Mastering Zephyr Driver Development"
talk given at Zephyr Developer Summit 2022.

The application is a prototype of a _smart_ lock. It allows to open/close a lock
by both using a fingerprint reader and BLE. It is based on the official
[example-application][example-application] and it tries to illustrate
the most relevant concepts used when developing Zephyr drivers.

https://user-images.githubusercontent.com/25011557/168632171-69db841b-a3d1-43d3-8da5-193831f6ca90.mp4

[example-application]: https://github.com/zephyrproject-rtos/example-application

## Getting Started

Before getting started, make sure you have a proper Zephyr development
environment. You can follow the official
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

### Initialization

The first step is to initialize the workspace folder where the
`zds-2022-drivers-app` and needed Zephyr modules will be cloned. You can do
that by running:

```shell
# initialize workspace for the zds-2022-drivers-app (main branch)
west init -m https://github.com/teslabs/zds-2022-drivers-app --mr main zds-2022-drivers-app
# update Zephyr modules
cd zds-2022-drivers-app
west update
```

### Build & Run

The application can be built by running:

```shell
west build -b nrf52840dk_nrf52840 -s app
```

Note that any other board may be used if an appropriate overlay is provided (see
`app/boards`).

Once you have built the application you can flash it by running:

```shell
west flash
```