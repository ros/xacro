# Bazel

This directory contains support files and tests for the bazel build system.

In addition to supporting building xacro with bazel, this also introduces two rules for build-time generation of xacro files.

## xacro_file

Allows you to transform a single xacro file into a generated output

A simple example:

```
load("@xacro//bazel:defs.bzl", "xacro_file")

# By default, will transform input filename with .xacro removed
xacro_file(
  name = "sample1",
  src = "sample1.xml.xacro",
)
```

A more complex example:

```
load("@xacro//bazel:defs.bzl", "xacro_file")

# By default, will transform input filename with .xacro removed
xacro_file(
  name = "complex_example",
  src = "complex.xml.xacro",
  # Override the default output file name
  out = "my_complex_model.xml",
  # Depend on the XML file that we generated in the previous step
  deps = [":sample1"],
  # Set extra substitution args via the command line
  extra_args = ["special_argument:=foo"]
)
```

Note in the case of the more complex example, you can use bazel-specified filenames if they are specified in the `deps` field:

```
<?xml version="1.0"?>
<root xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- include a file from a bazel path -->
  <xacro:include filename="//sample1.xml"/>
</root>
```

## xacro_filegroup

Allows you to transform multiple xacro files into a generated filegroup

```
xacro_filegroup(
    name = "samples",
    srcs = [
        "sample1.xml.xacro",
        "sample2.xml.xacro",
    ],
    data = [
        "box.xml",
    ],
)
```
