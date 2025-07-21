use dora_node_api::{self, DoraNode};
use eyre::{bail, Context};

fn main() -> eyre::Result<()> {
    let mut operator = DoraNode::init_from_env()?;

    let inputs = operator.inputs()?;

    while let Ok(input) = inputs.recv() {
        match input.id.as_str() {
            "message" => {
                let data = input.data();
                let received_string =
                    std::str::from_utf8(&data).wrap_err("received message was not utf8-encoded")?;
                println!("received message: {}", received_string);
                if !received_string.starts_with("operator received random value ") {
                    bail!("unexpected message format (should start with 'operator received random value')")
                }
                if !received_string.ends_with(" ticks") {
                    bail!("unexpected message format (should end with 'ticks')")
                }
            }
            other => eprintln!("Ignoring unexpected input `{other}`"),
        }
    }

    Ok(())
}
