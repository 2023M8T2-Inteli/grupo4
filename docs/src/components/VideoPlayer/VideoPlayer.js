import React from 'react';
import { ReactPlayer } from 'react-player';

const VideoPlayer = ({ url }) => (
  <ReactPlayer playing controls url={url} />
);

export default VideoPlayer;