use std::{env, fs::File, io::Write, path::PathBuf};

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());

    // esp-bootloader-esp-idf 0.5.0 places the app descriptor in .flash.appdesc,
    // but esp-hal 1.0.0 linker scripts only include .rodata_desc. Insert the section
    // before .rodata_desc so it lands at RODATA origin (partition_start + 0x20),
    // which is exactly where the ESP-IDF bootloader expects it.
    let script = b"\
SECTIONS {\n\
  .flash.appdesc : ALIGN(4) {\n\
    KEEP(*(.flash.appdesc));\n\
    KEEP(*(.flash.appdesc.*));\n\
  } > RODATA\n\
}\n\
INSERT BEFORE .rodata_desc;\n";

    File::create(out.join("flash_appdesc.x"))
        .unwrap()
        .write_all(script)
        .unwrap();

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg=-Tflash_appdesc.x");
    println!("cargo:rerun-if-changed=build.rs");
}
