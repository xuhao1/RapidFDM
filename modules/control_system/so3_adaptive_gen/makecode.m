function makecode
codegen L1Step.prj -o L1AircraftControl -d codegen/lib/L1AircraftControl
system('./copy2px4.sh')
end