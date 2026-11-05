Place the exported S10+Piper ONNX policy in this directory.

The only retained policy is:

`dwbc_s10_piper_kp60_wheelonly_v16_41000.onnx`

Required graph contract:

- input name: `obs`, shape `[batch, 836]`
- output name: `actions`, shape `[batch, 22]`
