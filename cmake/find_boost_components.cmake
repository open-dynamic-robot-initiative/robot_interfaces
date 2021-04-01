# Put this in a separate file, so it can be experted via CONFIG_EXTRAS.
# This is needed because ament doesn't support exporting components (see
# https://answers.ros.org/question/331089/ament_export_dependenciesboost-not-working/?answer=332460#post-id-332460)
find_package(Boost REQUIRED COMPONENTS iostreams)
