use dora_core::{
    config::OperatorId,
    descriptor::{Descriptor, DescriptorExt, NodeExt, SINGLE_OPERATOR_DEFAULT_ID},
};
use dora_message::descriptor::EnvValue;
use eyre::{eyre, Context};
use std::{collections::BTreeMap, path::Path, process::Command};

use crate::resolve_dataflow;

pub fn build(dataflow: String, uv: bool) -> eyre::Result<()> {
    let dataflow = resolve_dataflow(dataflow).context("could not resolve dataflow")?;
    let descriptor = Descriptor::blocking_read(&dataflow)?;
    let dataflow_absolute = if dataflow.is_relative() {
        std::env::current_dir().unwrap().join(dataflow)
    } else {
        dataflow.to_owned()
    };
    let working_dir = dataflow_absolute.parent().unwrap();

    let default_op_id = OperatorId::from(SINGLE_OPERATOR_DEFAULT_ID.to_string());

    for node in descriptor.nodes {
        match node.kind()? {
            dora_core::descriptor::NodeKind::Standard(_) => {
                run_build_command(node.build.as_deref(), working_dir, uv, node.env.clone())
                    .with_context(|| {
                        format!("build command failed for standard node `{}`", node.id)
                    })?
            }
            dora_core::descriptor::NodeKind::Runtime(runtime_node) => {
                for operator in &runtime_node.operators {
                    run_build_command(
                        operator.config.build.as_deref(),
                        working_dir,
                        uv,
                        node.env.clone(),
                    )
                    .with_context(|| {
                        format!(
                            "build command failed for operator `{}/{}`",
                            node.id, operator.id
                        )
                    })?;
                }
            }
            dora_core::descriptor::NodeKind::Custom(custom_node) => run_build_command(
                custom_node.build.as_deref(),
                working_dir,
                uv,
                node.env.clone(),
            )
            .with_context(|| format!("build command failed for custom node `{}`", node.id))?,
            dora_core::descriptor::NodeKind::Operator(operator) => run_build_command(
                operator.config.build.as_deref(),
                working_dir,
                uv,
                node.env.clone(),
            )
            .with_context(|| {
                format!(
                    "build command failed for operator `{}/{}`",
                    node.id,
                    operator.id.as_ref().unwrap_or(&default_op_id)
                )
            })?,
        }
    }

    Ok(())
}

fn run_build_command(
    build: Option<&str>,
    working_dir: &Path,
    uv: bool,
    envs: Option<BTreeMap<String, EnvValue>>,
) -> eyre::Result<()> {
    if let Some(build) = build {
        let lines = build.lines().collect::<Vec<_>>();
        for build_line in lines {
            let mut split = build_line.split_whitespace();

            let program = split
                .next()
                .ok_or_else(|| eyre!("build command is empty"))?;
            let mut cmd = if uv && (program == "pip" || program == "pip3") {
                let mut cmd = Command::new("uv");
                cmd.arg("pip");
                cmd
            } else {
                Command::new(program)
            };
            cmd.args(split);

            // Inject Environment Variables
            if let Some(envs) = envs.clone() {
                for (key, value) in envs {
                    let value = value.to_string();
                    cmd.env(key, value);
                }
            }

            cmd.current_dir(working_dir);
            let exit_status = cmd
                .status()
                .wrap_err_with(|| format!("failed to run `{}`", build))?;
            if !exit_status.success() {
                return Err(eyre!("build command `{build_line}` returned {exit_status}"));
            }
        }
        Ok(())
    } else {
        Ok(())
    }
}
