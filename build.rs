use std::process::Command;

fn main() {
    if cfg!(target_os = "windows") {
        // Clean the build directory
        Command::new("cmd")
            .args(&["/C", "cd f16_model && rmdir /S /Q build && mkdir build"])
            .status()
            .expect("Failed to clean build directory");

        // Create and navigate to the build directory
        Command::new("cmd")
            .args(&[
                "/C",
                "cd f16_model\\build && cmake .. && cmake --build . --target install",
            ])
            .status()
            .expect("Failed to build f16_model, maybe you need to install cmake and make");

        // Create models directory
        Command::new("cmd")
            .args(&[
                "/C",
                "mkdir models && xcopy f16_model\\install\\* models /E /I /Y",
            ])
            .status()
            .expect("Failed to create models directory");
    } else {
        // Clean the build directory
        Command::new("sh")
            .arg("-c")
            .arg("cd f16_model && rm -rf build && mkdir -p build")
            .status()
            .expect("Failed to clean build directory");

        // Create and navigate to the build directory
        Command::new("sh")
            .arg("-c")
            .arg("cd f16_model/build && cmake .. && make install")
            .status()
            .expect("Failed to build f16_model, maybe you need to install cmake and make");

        // Create models directory
        Command::new("sh")
            .arg("-c")
            .arg("mkdir -p models && cp -r f16_model/install/* models")
            .status()
            .expect("Failed to create models directory");
    }

    println!("cargo:rerun-if-changed=f16_model");
}
