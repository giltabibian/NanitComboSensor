mod abi;
mod app;
mod ssh;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_maximized(true)
            .with_inner_size([1400.0, 950.0]),
        ..Default::default()
    };
    eframe::run_native(
        "envcombo-ctl",
        options,
        Box::new(|_cc| Ok(Box::new(app::EnvComboCtl::default()))),
    )
}
