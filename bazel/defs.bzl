"""Provider and rules for generating xacro output at build time."""

XacroInfo = provider(
    "Provider holding the result of a xacro generation step.",
    fields = ["result"],
)

XACRO_EXTENSION = ".xacro"

def _xacro_impl(ctx):
    # If user doesn't define output, use the input minus the xacro extension
    out = ctx.outputs.out or ctx.actions.declare_file(ctx.file.src.path[:-len(XACRO_EXTENSION)])

    # Compute prefix in the case that this is in an external module
    prefix = "external/" + ctx.label.repo_name + "/"

    # Create a temporary directory in the current location in the tree
    temp_dir = "TMP_XACRO/" + ctx.label.name

    # Gather inputs for the xacro command
    direct_inputs = [ctx.file.src] + ctx.files.data
    dep_inputs = [dep[XacroInfo].result for dep in ctx.attr.deps]

    # For each direct input, symlink the input into the temporary directory
    symlink_paths = []
    for input in direct_inputs:
        input_path = input.path
        if input_path.startswith(prefix):
            input_path = input_path[len(prefix):]
        symlink_path = ctx.actions.declare_file(temp_dir + "/" + input_path)
        ctx.actions.symlink(
            output = symlink_path,
            target_file = input,
        )
        symlink_paths.append(symlink_path)

    # For each dependent input, symlink the input into the temporary directory
    for di in dep_inputs:
        symlink_path = ctx.actions.declare_file(temp_dir + "/" + di.short_path)
        ctx.actions.symlink(
            output = symlink_path,
            target_file = di,
        )
        symlink_paths.append(symlink_path)

    # If the input path is prefixed (that is, we are an external module),
    # then reflect that in the build paths and root directory
    input_path = ctx.file.src.path
    if input_path.startswith(prefix):
        input_path = input_path[len(prefix):]

    if out.path.endswith(out.short_path):
        # Internal module
        output_path = out.path[:-len(out.short_path)]
    else:
        # This is the case that we are an external module
        output_path = out.dirname
    root_dir = output_path + '/' + temp_dir

    arguments = [
        "-o",
        out.path,
        "--root-dir",
        root_dir,
        input_path
    ]
    arguments += ["{}:={}".format(arg, val) for arg, val in ctx.attr.arguments.items()]

    ctx.actions.run(
        inputs = symlink_paths,
        outputs = [out],
        arguments = arguments,
        executable = ctx.executable._xacro,
        progress_message = "Running xacro: %s -> %s" % (ctx.file.src.short_path, out.short_path),
        mnemonic = "Xacro",
    )

    return [
        XacroInfo(result = out),
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
        "arguments": attr.string_dict(),
        "deps": attr.label_list(providers = [XacroInfo]),
        "_xacro": attr.label(
            default = Label("//:xacro"),
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
