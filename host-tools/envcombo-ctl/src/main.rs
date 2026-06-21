mod abi;
mod app;
mod ssh;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "envcombo-ctl",
        options,
        Box::new(|_cc| Ok(Box::new(app::EnvComboCtl::default()))),
    )
}
