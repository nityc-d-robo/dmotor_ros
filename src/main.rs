use safe_drive:: {
    context::Context, error::DynError, logger::Logger, pr_info, topic::subscriber::Subscriber,
};
use drobo_interfaces::msg::{MdLibMsg, SdLibMsg};
use rusb::{DeviceHandle, GlobalContext};
use motor_lib;

use std::sync::LazyLock;

static HANDLE: LazyLock<DeviceHandle<GlobalContext>> = LazyLock::new(|| motor_lib::init_usb_handle(0x483, 0x5740, 1));

#[async_std::main]
async fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;
    let node = ctx.create_node("d_motor_ros", None, Default::default())?;
    let subscriber1 = node.create_subscriber::<MdLibMsg>("md_driver_topic", None)?;
    let task1 = async_std::task::spawn(md_receiver(subscriber1, &HANDLE));
    task1.await?;
    Ok(())
}

async fn md_receiver(mut subscriber: Subscriber<MdLibMsg>, handle_: &DeviceHandle<GlobalContext>) -> Result<(), DynError> {
    loop{
        let msg = subscriber.recv().await?;
        let status = match msg.mode{
            motor_lib::md::Mode::PWM => motor_lib::md::send_pwm(handle_, msg.address,  if msg.phase {msg.power} else {-1 * msg.power}),
            motor_lib::md::Mode::SPEED => motor_lib::md::send_speed(handle_, msg.address, msg.power),
            motor_lib::md::Mode::ANGLE => motor_lib::md::send_angle(handle_, msg.address, msg.angle),
            _ => motor_lib::md::receive_status(handle_, msg.address)
        };
    }
    Ok(())
}