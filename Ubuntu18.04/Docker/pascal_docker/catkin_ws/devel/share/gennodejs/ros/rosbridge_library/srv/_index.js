
"use strict";

let TestMultipleRequestFields = require('./TestMultipleRequestFields.js')
let TestRequestAndResponse = require('./TestRequestAndResponse.js')
let TestResponseOnly = require('./TestResponseOnly.js')
let TestMultipleResponseFields = require('./TestMultipleResponseFields.js')
let AddTwoInts = require('./AddTwoInts.js')
let TestArrayRequest = require('./TestArrayRequest.js')
let TestEmpty = require('./TestEmpty.js')
let TestRequestOnly = require('./TestRequestOnly.js')
let TestNestedService = require('./TestNestedService.js')
let SendBytes = require('./SendBytes.js')

module.exports = {
  TestMultipleRequestFields: TestMultipleRequestFields,
  TestRequestAndResponse: TestRequestAndResponse,
  TestResponseOnly: TestResponseOnly,
  TestMultipleResponseFields: TestMultipleResponseFields,
  AddTwoInts: AddTwoInts,
  TestArrayRequest: TestArrayRequest,
  TestEmpty: TestEmpty,
  TestRequestOnly: TestRequestOnly,
  TestNestedService: TestNestedService,
  SendBytes: SendBytes,
};
