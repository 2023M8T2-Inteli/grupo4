import React from 'react'
import Image from 'next/image'
import { useRouter } from 'next/navigation';

const Sidebar = () => {
  const router = useRouter();
  const goToUsers = () => {
    // Replace '/users' with the path you want to redirect to
    router.push('/users');
  };
  const goHome = () => {
    router.push('/');
  }
  const goQRCode = () => {
    router.push('/qrcode')
  }

  const goToTools = () => {
    router.push('/tools');
  }

  const goToPoints = () => {
    router.push('/locations');
  }



  return (
    <div className='w-64 h-screen bg-cover bg-center relative'
    style={{ backgroundImage: `url('https://d17sdup6iumur7.cloudfront.net/bg.png')` }}>
        <h1 className='text-5xl text-white text-bold text-center p-4'>VALLET</h1>
        <div className='flex flex-col text-white p-6 mb-2'>
            <button className='p-2' onClick={goHome}>Home</button>
            <button className='p-2' onClick={goQRCode}>QR code</button>
            <button className='p-2' onClick={goToUsers} >Usuários</button>
            <button className='p-2' onClick={goToTools} >Itens</button>
            <button className='p-2' onClick={goToPoints} >Destinos</button>
        </div>
        <img className='absolute bottom-0' src={'https://d17sdup6iumur7.cloudfront.net/robot.png'} width={500} height={500} />

    </div>
  )
}

export default Sidebar
