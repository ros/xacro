"""Provider and rules for generating xacro output at build time."""

XacroInfo = provider(
  "Results of the xacro generation step"
  fields = ["result", "data"]
)
