"""Provider and rules for generating xacro output at build time."""

XacroInfo = provider(
    "Provider holding the result of a xacro generation step.",
    fields = ["result", "data"],
)

XACRO_EXTENSION = ".xacro"

def _xacro_impl(ctx):
    if ctx.outputs.out:
        out = ctx.outputs.out
    else:
        src = ctx.file.src.basename
        if not src.endswith(XACRO_EXTENSION):
            fail("xacro_file src should be named *.xacro not {}".format(src))
        out = ctx.actions.declare_file(src[:-len(XACRO_EXTENSION)])

    # The list of arguments we pass to the script.
    args = [ctx.file.src.path, "-o", out.path] + ctx.attr.extra_args

    # Action to call the script.
    all_inputs = [ctx.file.src] + ctx.files.data + [dep[XacroInfo].result for dep in ctx.attr.deps]
    ctx.actions.run(
        inputs = all_inputs,
        outputs = [out],
        arguments = args,
        env = {"XACRO_INPUTS": "\n".join([file.path for file in all_inputs])},
        progress_message = "Running xacro: %s -> %s" % (ctx.file.src.short_path, out.short_path),
        executable = ctx.executable._xacro,
    )

    xacro_data = depset(
        direct = [out] + ctx.files.data + [d[XacroInfo].result for d in ctx.attr.deps],
        transitive = [d[XacroInfo].data for d in ctx.attr.deps],
    )

    runfiles = ctx.runfiles(files = xacro_data.to_list())

    return [
        XacroInfo(result = out, data = xacro_data),
        DefaultInfo(
            files = depset([out]),
            data_runfiles = ctx.runfiles(files = [out]),
        ),
    ]

xacro_file = rule(
    attrs = {
        "src": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
        "out": attr.output(),
        "data": attr.label_list(
            allow_files = True,
        ),
        "extra_args": attr.string_list(),
        "deps": attr.label_list(providers = [XacroInfo]),
        "_xacro": attr.label(
            default = "@xacro//:xacro",
            cfg = "host",
            executable = True,
        ),
    },
    implementation = _xacro_impl,
    provides = [XacroInfo, DefaultInfo],
)

def xacro_filegroup(
        name,
        srcs = [],
        data = [],
        tags = [],
        visibility = None):
    """Runs xacro on several input files, creating a filegroup of the output.

    The output filenames will match the input filenames but with the ".xacro"
    suffix removed.

    Xacro is the ROS XML macro tool; http://wiki.ros.org/xacro.

    Args:
      name: The name of the filegroup label.
      srcs: The xacro input files of this rule.
      data: Optional supplemental files required by the srcs.
    """
    outs = []
    for src in srcs:
        if not src.endswith(XACRO_EXTENSION):
            fail("xacro_filegroup srcs should be named *.xacro not {}".format(
                src,
            ))
        out = src[:-len(XACRO_EXTENSION)]
        outs.append(out)
        xacro_file(
            name = out,
            src = src,
            data = data,
            tags = tags,
            visibility = ["//visibility:private"],
        )
    native.filegroup(
        name = name,
        srcs = outs,
        visibility = visibility,
    )
