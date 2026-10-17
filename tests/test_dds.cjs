// Original pixel expectations exercise the portable decoder without a GPU or game files.
const test = require('node:test');
const assert = require('node:assert/strict');
const {decodeLevel} = require('../tools/prototype/viewer/dds.js');
function decode(format, bytes, width=4, height=4, extra={}) {
  const buffer = Uint8Array.from(bytes).buffer;
  return decodeLevel({format, levels:[{width,height,offset:0,length:bytes.length,...extra.level}], ...extra}, buffer);
}
const solidRed = [0,248,224,7,0,0,0,0];
test('BC1 expands endpoints and interpolated colours', () => {
  const result=decode('BC1',[0,248,224,7,0xe4,0,0,0]);
  assert.deepEqual([...result.slice(0,16)], [255,0,0,255, 0,255,0,255, 170,85,0,255, 85,170,0,255]);
});
test('BC1 transparent branch and cropped blocks', () => {
  assert.deepEqual([...decode('BC1',[0,0,255,255,255,255,255,255],1,1)], [0,0,0,0]);
  assert.equal(decode('BC1',[...solidRed,...solidRed],5,3).length,60);
});
test('BC2 explicit alpha uses four-colour interpolation even for ascending endpoints', () => {
  const bytes=[0x10,0x32,0x54,0x76,0x98,0xba,0xdc,0xfe,0,0,255,255,0xaa,0xaa,0xaa,0xaa];
  const result=decode('BC2',bytes);
  assert.deepEqual([...result.slice(0,8)], [85,85,85,0,85,85,85,17]);
  assert.equal(result[63],255);
});
function bc3(a,b,indices) {
  let packed=0n;
  indices.forEach((value,index)=>packed |= BigInt(value) << BigInt(index*3));
  const alpha=Array.from({length:6},(_,i)=>Number((packed >> BigInt(i*8))&255n));
  return decode('BC3',[a,b,...alpha,...solidRed]);
}
test('BC3 both interpolation branches and high alpha-index bits', () => {
  const result=bc3(210,0,[0,1,2,3,4,5,6,7,0,1,2,3,4,5,6,7]);
  assert.deepEqual(Array.from({length:8},(_,i)=>result[i*4+3]), [210,0,180,150,120,90,60,30]);
  assert.equal(result[63],30);
  const second=bc3(0,200,[2,3,4,5,6,7]);
  assert.deepEqual(Array.from({length:6},(_,i)=>second[i*4+3]), [40,80,120,160,0,255]);
});
test('Raw BGR rows preserve ordering, padding and alpha', () => {
  const result=decode('RGB',[0,0,255,0,255,0,9,9,255,0,0,255,255,255,9,9],2,2,
    {bits:24,masks:[0xff0000,0xff00,0xff,0],level:{row_pitch:8}});
  assert.deepEqual([...result], [255,0,0,255,0,255,0,255,0,0,255,255,255,255,255,255]);
  const la=decode('LUMINANCE',[64,128],1,1,{bits:16,masks:[255,0,0,65280],level:{row_pitch:2}});
  assert.deepEqual([...la],[64,64,64,128]);
  const rgba=decode('RGB',[1,2,3,128],1,1,{bits:32,masks:[255,65280,16711680,4278190080],level:{row_pitch:4}});
  assert.deepEqual([...rgba],[1,2,3,128]);
});
test('Malformed block sizes and hostile dimensions fail before allocation', () => {
  assert.throws(()=>decode('BC1',[1,2]),/block length/);
  assert.throws(()=>decode('BC1',solidRed,999999,999999),/bounds/);
});
test('Raw RGB565 masks scale channels independently', () => {
  const result=decode('RGB',[0,248],1,1,{bits:16,masks:[0xf800,0x7e0,0x1f,0],level:{row_pitch:2}});
  assert.deepEqual([...result],[255,0,0,255]);
});
