use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    // Configure flash origin based on whether running standalone (0x08000000)
    // or behind BareBoot / Katapult bootloader (0x08002000).
    let memory_x = if env::var("CARGO_FEATURE_BOOTLOADER").is_ok() {
        b"MEMORY
{
  FLASH : ORIGIN = 0x08002000, LENGTH = 120K
  RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}
"
    } else {
        b"MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 128K
  RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}
"
    };

    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(memory_x)
        .unwrap();

    // Generate an empty defmt.x if missing, ensuring clean linking even when
    // invoked from workspace directories configured with global defmt linker flags.
    let defmt_x = out.join("defmt.x");
    if !defmt_x.exists() {
        let _ = File::create(defmt_x).and_then(|mut f| f.write_all(b"SECTIONS { }"));
    }

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_BOOTLOADER");
}
