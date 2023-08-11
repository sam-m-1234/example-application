mkdir workspace
python3 -m venv ~/workspace/.venv
. ~/workspace/.venv/bin/activate
pip install west
west init ~/workspace
cd workspace
west update
west zephyr-export
pip install -r ~/workspace/zephyr/scripts/requirements.txt

. ~/workspace/.venv/bin/activate && cd ~/workspace && west zephyr-export
west build -b qemu_x86 gateway-lower/app -p
west build -t run -- -DCONFIG_QEMU_ICOUNT=n
west build -b nrf5340dk_nrf5340_cpuapp gateway-lower/app -p

west build -b qemu_x86 gateway-lower/app -t run -- -DCONFIG_QEMU_ICOUNT=n

west build -t menuconfig