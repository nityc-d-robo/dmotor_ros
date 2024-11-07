use safe_drive:: {
    context::Context, error::DynError, logger::Logger, pr_error, topic::subscriber::Subscriber,
};
use drobo_interfaces::msg::{MdLibMsg, SmdLibMsg, BlMdLibMsg};
use rusb::{DeviceHandle, GlobalContext};
use motor_lib;

use std::sync::LazyLock;

static HANDLE: LazyLock<DeviceHandle<GlobalContext>> = LazyLock::new(|| motor_lib::init_usb_handle(0x483, 0x5740, 1).unwrap());

#[async_std::main]
async fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;
    let node = ctx.create_node("d_motor_ros", None, Default::default())?;
    let md_subscriber = node.create_subscriber::<MdLibMsg>("md_driver_topic", None)?;
    let smd_subscriber = node.create_subscriber::<SmdLibMsg>("smd_driver_topic", None)?;
    let blmd_subscriber = node.create_subscriber::<BlMdLibMsg>("blmd_driver_topic", None)?;
    let md_task = async_std::task::spawn(md_receiver(md_subscriber, &HANDLE));
    let smd_task = async_std::task::spawn(smd_receiver(smd_subscriber, &HANDLE));
    let blmd_task = async_std::task::spawn(blmd_receiver(blmd_subscriber, &HANDLE));
    
    md_task.await?;
    blmd_task.await?;
    Ok(())
}

async fn md_receiver(mut subscriber: Subscriber<MdLibMsg>, handle_: &DeviceHandle<GlobalContext>) -> Result<(), DynError> {
    let logger = Logger::new("md_receiver");
    loop{
        let msg = subscriber.recv().await?;
        let status = loop {
            match match msg.mode {
                motor_lib::md::Mode::PWM => motor_lib::md::send_pwm(handle_, msg.address,  if msg.phase {msg.power as i16} else {-1 * msg.power as i16}),
                motor_lib::md::Mode::SPEED => motor_lib::md::send_speed(handle_, msg.address, msg.power as i16),
                motor_lib::md::Mode::ANGLE => motor_lib::md::send_angle(handle_, msg.address, msg.angle as i16),
                _ => motor_lib::md::receive_status(handle_, msg.address)
            } {  
                Ok(t) => break t,
                Err(e) => {
                    pr_error!(logger, "{:?}", e);
                }
            }
        };
    }
}

async fn smd_receiver(mut subscriber: Subscriber<SmdLibMsg>, handle_: &DeviceHandle<GlobalContext>) -> Result<(), DynError> {
    let logger = Logger::new("smd_receiver");
    loop{
        let msg = subscriber.recv().await?;
        let status = loop {
            match match msg.mode {
                motor_lib::smd::Mode::ANGLE => motor_lib::smd::send_angle(handle_, msg.address, msg.port,msg.angle),
                _ => motor_lib::smd::receive_status(handle_, msg.address)
            } {
                Ok(t) => break t,
                Err(e) => {
                    pr_error!(logger, "{:?}", e);
                }
            };
        };
    }
}

async fn blmd_receiver(mut subscriber: Subscriber<BlMdLibMsg>, handle_: &DeviceHandle<GlobalContext>) -> Result<(), DynError> {
    let logger = Logger::new("blmd_receiver");
    loop{
        let msg = subscriber.recv().await?;
        let status = loop {
            match match msg.mode{
            motor_lib::blmd::Mode::CURRENT => motor_lib::blmd::send_current(handle_, msg.controller_id,  msg.current),
            _ => motor_lib::blmd::receive_status(handle_, msg.controller_id)
            } {
                Ok(t) => break t,
                Err(e) => {
                    pr_error!(logger, "{:?}", e);
                }
            };
        };
    }
}