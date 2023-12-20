// import { HandlerBase } from './base';
//
// type RegexFunctionMapping = Map<RegExp, (matches: RegExpMatchArray) => void>;
//
// export class UserMessageHandler extends HandlerBase {
//   protected intent_dict: RegexFunctionMapping;
//   constructor() {
//     super();
//     this.intent_dict = new Map([
//       [
//         /\b([Nn]ova)|([Pp]eça)\b/gi,
//         (matches) => {
//           console.log('New Order:', matches[0]);
//         },
//       ],
//       [
//         /\b([Ss]tatus)|([Pp]edido)\b/gi,
//         (matches) => {
//           console.log('Status Order:', matches[0]);
//         },
//       ],
//       [
//         /\b([Pp]edidos)|([Aa]berto)|([Aa]bertos)\b/gi,
//         (matches) => {
//           console.log('Open Orders:', matches[0]);
//         },
//       ],
//       [
//         /\b([Cc]ancelar)|([Pp]edido)\b/gi,
//         (matches) => {
//           console.log('Cancel Order:', matches[0]);
//         },
//       ],
//       [
//         /\b([Ff]alar)|([Aa]tendente)|([Cc]om)\b/gi,
//         (matches) => {
//           console.log('Contact:', matches[0]);
//         },
//       ],
//       [
//         /\b([Aa]lterar)|([Nn]ome)|([Cc]adastrado)\b/gi,
//         (matches) => {
//           console.log('Change Name:', matches[0]);
//         },
//       ],
//     ]);
//   }
//
//   protected handleRequestMenu = async () => {
//     try {
//       this.sendMenu();
//       await userService.updateRequestUser(message.from, 2);
//     } catch (error: any) {
//       console.error('An error occured', error);
//       message.reply(
//         'An error occured, please contact the administrator. (' +
//           error.message +
//           ')',
//       );
//     }
//   };
// }
