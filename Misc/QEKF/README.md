If the generated code in the `MATLABCoder` folder is updated, please remember to add the optimization attribute to the start of the function (before void) in the `cpp` file

```cpp
__attribute__((optimize("O3"))) void ....
```
