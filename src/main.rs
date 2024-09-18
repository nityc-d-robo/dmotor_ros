use safe_drive:: {
    context::Context, error::DynError, logger::Logger, pr_info, topic::subscriber::Subscriber,
};
use drobo_interfaces::msg::{MdLibMsg, SdLibMsg, BlMdLibMsg};
use rusb::{DeviceHandle, GlobalContext};
use motor_lib;

use std::sync::LazyLock;

static HANDLE: LazyLock<DeviceHandle<GlobalContext>> = LazyLock::new(|| motor_lib::init_usb_handle(0x483, 0x5740, 1));

#[async_std::main]
async fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;
    let node = ctx.create_node("d_motor_ros", None, Default::default())?;
    let md_subscriber = node.create_subscriber::<MdLibMsg>("md_driver_topic", None)?;
    let sd_subscriber = node.create_subscriber::<SdLibMsg>("sd_driver_topic", None)?;
    let blmd_subscriber = node.create_subscriber::<BlMdLibMsg>("blmd_driver_topic", None)?;
    let md_task = async_std::task::spawn(md_receiver(md_subscriber, &HANDLE));
    let sd_task = async_std::task::spawn(sd_receiver(sd_subscriber, &HANDLE));
    let blmd_task = async_std::task::spawn(blmd_receiver(blmd_subscriber, &HANDLE));
    
    md_task.await?;
    sd_task.await?;
    blmd_task.await?;
    Ok(())
}

async fn md_receiver(mut subscriber: Subscriber<MdLibMsg>, handle_: &DeviceHandle<GlobalContext>) -> Result<(), DynError> {
    loop{
        let msg = subscriber.recv().await?;
        let _ = match msg.mode{
            motor_lib::md::Mode::PWM => motor_lib::md::send_pwm(handle_, msg.address,  if msg.phase {msg.power as i16} else {-1 * msg.power as i16}),
            motor_lib::md::Mode::SPEED => motor_lib::md::send_speed(handle_, msg.address, msg.power as i16),
            motor_lib::md::Mode::ANGLE => motor_lib::md::send_angle(handle_, msg.address, msg.angle as i16),
            _ => motor_lib::md::receive_status(handle_, msg.address)
        };
    }
}
async fn sd_receiver(mut subscriber: Subscriber<SdLibMsg>, handle_: &DeviceHandle<GlobalContext>) -> Result<(), DynError> {
    loop{
        let msg = subscriber.recv().await?;
        let _ = motor_lib::sd::send_power(handle_, msg.address, 0, msg.power1 as i16);
    }
}
async fn blmd_receiver(mut subscriber: Subscriber<BlMdLibMsg>, handle_: &DeviceHandle<GlobalContext>) -> Result<(), DynError> {
    loop{
        let msg = subscriber.recv().await?;
        let _ = match msg.mode{
            motor_lib::blmd::Mode::CURRENT => motor_lib::blmd::send_current(handle_, msg.controller_id,  msg.current),
            _ => motor_lib::blmd::receive_status(handle_, msg.controller_id)
        };
    }
}