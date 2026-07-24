# usteer

usteer is a client steering daemon written for OpenWrt.

Its goal is to optimize roaming behavior of wireless clients (STAs) in a ESS consisting of multiple BSS / APs.

## Functions

 - Synchronization of Neighbor Reports between multiple APs
 - Policy-based decisions for probe- / association- / authentication requests received from STAs
 - Requesting clients to roam to a different BSS based on SNR / signal-level
 - Channel-load based client steering to different BSS

## Installation

usteer is available from the OpenWrt packages feed and can be installed on devices running OpenWrt 21.02+ using opkg:

```
opkg update; opkg install usteer
```

You can find the complete documentation for setting everything up and getting it running here:[OpenWRT documentation](https://openwrt.org/docs/guide-user/network/wifi/usteer).

### Config recommendation
The default settings are the best in most environments (many users overdo optimizations to it) to support roaming and band steering. At best nothing has to be configured but take care that usteer communication is not done on a public network.


## Submitting patches

usteer patches are welcome on the openwrt-devel mailing list.

Before submitting patches, check out OpenWrts guide on submission policies.

Make sure to add a `usteer` subject prefix using the `--subject-prefix` option when exporting the patch with `git format-patch`.
